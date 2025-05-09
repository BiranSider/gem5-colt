/*
 * Copyright (c) 2012 ARM Limited
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "arch/riscv/pagetable_walker.hh"

#include <memory>

#include "arch/riscv/faults.hh"
#include "arch/riscv/page_size.hh"
#include "arch/riscv/pagetable.hh"
#include "arch/riscv/tlb.hh"
#include "base/bitfield.hh"
#include "base/trie.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/PageTableWalker.hh"
#include "mem/packet_access.hh"
#include "mem/request.hh"

namespace gem5
{

namespace RiscvISA {

Fault
Walker::start(ThreadContext * _tc, BaseMMU::Translation *_translation,
              const RequestPtr &_req, BaseMMU::Mode _mode)
{
    // TODO: in timing mode, instead of blocking when there are other
    // outstanding requests, see if this request can be coalesced with
    // another one (i.e. either coalesce or start walk)
    WalkerState * newState = new WalkerState(this, _translation, _req);
    newState->initState(_tc, _mode, sys->isTimingMode());
    if (currStates.size()) {
        assert(newState->isTiming());
        DPRINTF(PageTableWalker, "Walks in progress: %d\n", currStates.size());
        currStates.push_back(newState);
        return NoFault;
    } else {
        currStates.push_back(newState);
        Fault fault = newState->startWalk();
        if (!newState->isTiming()) {
            currStates.pop_front();
            delete newState;
        }
        return fault;
    }
}

Fault
Walker::startFunctional(ThreadContext * _tc, Addr &addr, unsigned &logBytes,
              BaseMMU::Mode _mode)
{
    funcState.initState(_tc, _mode);
    return funcState.startFunctional(addr, logBytes);
}

bool
Walker::WalkerPort::recvTimingResp(PacketPtr pkt)
{
    return walker->recvTimingResp(pkt);
}

bool
Walker::recvTimingResp(PacketPtr pkt)
{
    WalkerSenderState * senderState =
        dynamic_cast<WalkerSenderState *>(pkt->popSenderState());
    WalkerState * senderWalk = senderState->senderWalk;
    bool walkComplete = senderWalk->recvPacket(pkt);
    delete senderState;
    if (walkComplete) {
        std::list<WalkerState *>::iterator iter;
        for (iter = currStates.begin(); iter != currStates.end(); iter++) {
            WalkerState * walkerState = *(iter);
            if (walkerState == senderWalk) {
                iter = currStates.erase(iter);
                break;
            }
        }
        delete senderWalk;
        // Since we block requests when another is outstanding, we
        // need to check if there is a waiting request to be serviced
        if (currStates.size() && !startWalkWrapperEvent.scheduled())
            // delay sending any new requests until we are finished
            // with the responses
            schedule(startWalkWrapperEvent, clockEdge());
    }
    return true;
}

void
Walker::WalkerPort::recvReqRetry()
{
    walker->recvReqRetry();
}

void
Walker::recvReqRetry()
{
    std::list<WalkerState *>::iterator iter;
    for (iter = currStates.begin(); iter != currStates.end(); iter++) {
        WalkerState * walkerState = *(iter);
        if (walkerState->isRetrying()) {
            walkerState->retry();
        }
    }
}

bool Walker::sendTiming(WalkerState* sendingState, PacketPtr pkt)
{
    DPRINTF(PageTableWalker, "Send Tining\n");
    WalkerSenderState* walker_state = new WalkerSenderState(sendingState);
    pkt->pushSenderState(walker_state);
    if (port.sendTimingReq(pkt)) {
        DPRINTF(PageTableWalker, "Send Timing True\n");
        return true;
    } else {
        // undo the adding of the sender state and delete it, as we
        // will do it again the next time we attempt to send it
        pkt->popSenderState();
        delete walker_state;
        return false;
    }

}

Port &
Walker::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port")
        return port;
    else
        return ClockedObject::getPort(if_name, idx);
}

void
Walker::WalkerState::initState(ThreadContext * _tc,
        BaseMMU::Mode _mode, bool _isTiming)
{
    assert(state == Ready);
    started = false;
    tc = _tc;
    mode = _mode;
    timing = _isTiming;
    // fetch these now in case they change during the walk
    status = tc->readMiscReg(MISCREG_STATUS);
    pmode = walker->tlb->getMemPriv(tc, mode);
    satp = tc->readMiscReg(MISCREG_SATP);
    assert(satp.mode == AddrXlateMode::SV39);
}

void
Walker::startWalkWrapper()
{
    unsigned num_squashed = 0;
    WalkerState *currState = currStates.front();

    // check if we get a tlb hit to skip the walk
    Addr vaddr = Addr(sext<VADDR_BITS>(currState->req->getVaddr()));
    Addr vpn = getVPNFromVAddr(vaddr, currState->satp.mode);
    TlbEntry *e = tlb->lookup(vpn, currState->satp.asid, currState->mode,
                              true);
    Fault fault = NoFault;
    if (e) {
       fault = tlb->checkPermissions(currState->status, currState->pmode,
                                     vaddr, currState->mode, e->pte);
    }

    while ((num_squashed < numSquashable) && currState &&
           (currState->translation->squashed() || (e && fault == NoFault))) {
        currStates.pop_front();
        num_squashed++;

        DPRINTF(PageTableWalker, "Squashing table walk for address %#x\n",
            currState->req->getVaddr());

        // finish the translation which will delete the translation object
        if (currState->translation->squashed()) {
            currState->translation->finish(
                std::make_shared<UnimpFault>("Squashed Inst"),
                currState->req, currState->tc, currState->mode);
        } else {
            tlb->translateTiming(currState->req, currState->tc,
                                 currState->translation, currState->mode);
        }

        // delete the current request if there are no inflight packets.
        // if there is something in flight, delete when the packets are
        // received and inflight is zero.
        if (currState->numInflight() == 0) {
            delete currState;
        } else {
            currState->squash();
        }

        // check the next translation request, if it exists
        if (currStates.size()) {
            currState = currStates.front();
            vaddr = Addr(sext<VADDR_BITS>(currState->req->getVaddr()));
            Addr vpn = getVPNFromVAddr(vaddr, currState->satp.mode);
            e = tlb->lookup(vpn, currState->satp.asid, currState->mode,
                            true);
            if (e) {
               fault = tlb->checkPermissions(currState->status,
                                             currState->pmode, vaddr,
                                             currState->mode, e->pte);
            }
        } else {
            currState = NULL;
        }
    }
    if (currState && !currState->wasStarted()) {
        if (!e || fault != NoFault) {
            currState->startWalk();
        }
        else
            schedule(startWalkWrapperEvent, clockEdge(Cycles(1)));
    }
}

Fault
Walker::WalkerState::startWalk()
{
    Fault fault = NoFault;
    assert(!started);
    started = true;
    setupWalk(req->getVaddr());
    if (timing) {
        nextState = state;
        state = Waiting;
        timingFault = NoFault;
        sendPackets();
    } else {
        do {
            walker->port.sendAtomic(read);
            PacketPtr write = NULL;
            fault = stepWalk(write);
            assert(fault == NoFault || read == NULL);
            state = nextState;
            nextState = Ready;
            if (write)
                walker->port.sendAtomic(write);
        } while (read);
        state = Ready;
        nextState = Waiting;
    }
    return fault;
}

Fault
Walker::WalkerState::startFunctional(Addr &addr, unsigned &logBytes)
{
    Fault fault = NoFault;
    assert(!started);
    started = true;
    setupWalk(addr);

    do {
        walker->port.sendFunctional(read);
        // On a functional access (page table lookup), writes should
        // not happen so this pointer is ignored after stepWalk
        PacketPtr write = NULL;
        fault = stepWalk(write);
        assert(fault == NoFault || read == NULL);
        state = nextState;
        nextState = Ready;
    } while (read);
    logBytes = entry.logBytes;
    addr = entry.paddr << PageShift;

    return fault;
}

bool
Walker::isContaigous(int level, PTESv39 first_pte, PTESv39 second_pte)
{
    /* Simulates a gate-oriented comparision of two PTEs to detect if they
    are physically contagious
    */
    if (first_pte.perm == second_pte.perm && first_pte)
    {
        Addr first_ppn = (level == 1) ? first_pte.ppn1: first_pte.ppn2;
        Addr second_ppn = (level == 1) ? second_pte.ppn1: second_pte.ppn2;
        return second_ppn - first_ppn == 1;
    }
    return false;
}

union bitwize_casting {
    uint8_t asInt;
    bool asBits[8];
};

int8_t
Walker::indexedCoalesingEntryInformation(int level, PacketPtr &readInfo)
{
    /*
    Creates a contagious indicator for each PTE in the cacheline
    It does so in reference to the previous entry
    The assumption here is that in hardware you would create a defualt entry and mask it
    with the value from the index to "fix" it to the required entry in a bitwize operation
    */
    bitwize_casting readInfoCoalesingEntry;
    // The first one is the same as itself (reference)
    readInfoCoalesingEntry.asInt = 1;
    for (int i = 1; i < 8; i++)
    {
        PTESv39 first_pte = readInfo->getOffsetLE<uint64_t>(i-1);
        PTESv39 second_pte = readInfo->getOffsetLE<uint64_t>(i);
        readInfoCoalesingEntry.asInt += 
            this->isContaigous(level, first_pte, second_pte) ? (1 << i) : 0;
    }

    uint64_t readIndex = readInfo->getIdx();
    // Backward coalesing detection
    if (readIndex > 0) {
        for (int i = readIndex - 1; i > 0; i--)
        {
            // if it was already 0 it doesn't matter but only contaigous 1 would work
            readInfoCoalesingEntry.asBits[i] &= readInfoCoalesingEntry.asBits[i+1];
        }
    }
    // Forward coalesing detection
    if (readIndex < 7) {
        for (int i = readIndex + 1; i <= 7; i++)
        {
            readInfoCoalesingEntry.asBits[i] &= readInfoCoalesingEntry.asBits[i-1];
        }
    }
    // No matter what - validate the intended index itself.
    readInfoCoalesingEntry.asBits[readIndex] = 1;

    // At this point this contains the is valid in comparision to the index
    DPRINTF(PageTableWalker, "Coalesing Entry is: %d\n", readInfoCoalesingEntry.asInt);
    return readInfoCoalesingEntry.asInt;
}

PTESv39
Walker::getBaseCoalesingEntry(PacketPtr &readInfo, uint8_t coalesingData)
{
    bitwize_casting readInfoCoalesingEntry;
    readInfoCoalesingEntry.asInt = coalesingData;
    bitwize_casting lowestIndex;

    for (int i=0; i < 7; i++) {
        // We only want to preserve the lowest on bit for reference
        // The logic is Bi & (Bi ^ Bi+1) 
        /*
        Bi  |   Bi+1    |   Res
        0   |   0       |   0
        1   |   0       |   1
        0   |   1       |   0
        1   |   1       |   0
        One can assume an ADC would be used prior to a MUX that would be used
        to choose the base PTE
        */
       bool bi = readInfoCoalesingEntry.asBits[i];
       bool bi1 = readInfoCoalesingEntry.asBits[i+1];
       lowestIndex.asBits[i] = bi & (bi ^ bi1);
    }
    // Default to it because it's the case where None of these will be on
    int base_entry_index = 7;
    // Assume ADC logic
    for (int i=0; i < 7; i++) {
        if (lowestIndex.asBits[i]) {
            base_entry_index = i;
            break;
        } 
    }
    return readInfo->getOffsetLE<uint64_t>(base_entry_index);
}

// uint64_t
// Walker::getCoalesingLength(uint8_t coalesingData, ) {
//     /*
//     The final step in this thing is now that we map to a base PTE with a new size...

//     Understand how to:
//         1. TAG the data correctly (since it encompasses more pages)
//         2. Peresent the length information to through the TLB...
//     */
// }


Fault
Walker::WalkerState::stepWalk(PacketPtr &write)
{
    assert(state != Ready && state != Waiting);
    Fault fault = NoFault;
    write = NULL;

    PTESv39 pte = read->getOffsetLE<uint64_t>(read->getIdx());
    Addr nextRead = 0;
    bool doWrite = false;
    bool doTLBInsert = false;
    bool doEndWalk = false;


    DPRINTF(PageTableWalker, "[LEVEL%d] Buffer Start: %#x\tIndex: %d (%#x) -> PTE: %#x (r%dw%dv%d ppn %#x)\n",
        level, read->getAddr(), read->getIdx(), read->getAddr() + sizeof(PTESv39) * read->getIdx(), pte, pte.r, pte.w, pte.v, pte.ppn);
    // step 2:
    // Performing PMA/PMP checks on physical address of PTE

    // Let's check coalesing here
    DPRINTF(PageTableWalker, "Results:\n1[%#X] 2[%#X] 3[%#X] 4[%#X] \n5[%#X] 6[%#X] 7[%#X] 8[%#x]\n\tidx=%d\n",
         read->getOffsetLE<PTESv39>(0).ppn, read->getOffsetLE<PTESv39>(1).ppn, read->getOffsetLE<PTESv39>(2).ppn, read->getOffsetLE<PTESv39>(3).ppn, read->getOffsetLE<PTESv39>(4).ppn, read->getOffsetLE<PTESv39>(5).ppn, read->getOffsetLE<PTESv39>(6).ppn, read->getOffsetLE<PTESv39>(7).ppn, read->getIdx());


    // Effective privilege mode for pmp checks for page table
    // walks is S mode according to specs
    fault = walker->pmp->pmpCheck(read->req, BaseMMU::Read,
                    RiscvISA::PrivilegeMode::PRV_S, tc, entry.vaddr);

    if (fault == NoFault) {
        fault = walker->pma->check(read->req, BaseMMU::Read, entry.vaddr);
    }

    if (fault == NoFault) {
        // Here we need to detect a size... I think...
        /*
        Basically add logic for coalecing
        */



        // step 3:
        if (!pte.v || (!pte.r && pte.w)) {
            doEndWalk = true;
            DPRINTF(PageTableWalker, "PTE invalid, raising PF %#x v%d r%d w%d\n", pte, pte.v, pte.r, pte.w);
            fault = pageFault(pte.v);
        }
        else {
            // step 4:
            if (pte.r || pte.x) {
                // step 5: leaf PTE
                doEndWalk = true;
                fault = walker->tlb->checkPermissions(status, pmode,
                                                    entry.vaddr, mode, pte);

                // step 6
                if (fault == NoFault) {
                    if (level >= 1 && pte.ppn0 != 0) {
                        DPRINTF(PageTableWalker,
                                "PTE has misaligned PPN, raising PF\n");
                        fault = pageFault(true);
                    }
                    else if (level == 2 && pte.ppn1 != 0) {
                        DPRINTF(PageTableWalker,
                                "PTE has misaligned PPN, raising PF\n");
                        fault = pageFault(true);
                    }
                }

                if (fault == NoFault) {
                    // step 7
                    if (!pte.a) {
                        pte.a = 1;
                        doWrite = true;
                    }
                    if (!pte.d && mode == BaseMMU::Write) {
                        pte.d = 1;
                        doWrite = true;
                    }
                    // Performing PMA/PMP checks

                    if (doWrite) {

                        // this read will eventually become write
                        // if doWrite is True

                        fault = walker->pmp->pmpCheck(read->req,
                                            BaseMMU::Write, pmode, tc, entry.vaddr);

                        if (fault == NoFault) {
                            fault = walker->pma->check(read->req,
                                                BaseMMU::Write, entry.vaddr);
                        }

                    }
                    // perform step 8 only if pmp checks pass
                    if (fault == NoFault) {
                        DPRINTF(PageTableWalker,
                                "#0 leaf node at level %d, with vpn %#x\n",
                                 level, entry.vaddr);

                        // Now that we found the PTE - we want to perform coalesing logic
                        // walker->detectCoalesing(read, level);
                        int8_t coalesingData = walker->indexedCoalesingEntryInformation(level, read);
                        PTESv39 basePte = walker->getBaseCoalesingEntry(read, coalesingData);
                        entry.coalesingData = coalesingData;
                        // This isn't correct it's temporary :(

                        uint32_t numberOfPages = pte.ppn - basePte.ppn;
                        // step 8
                        entry.logBytes = numberOfPages + PageShift + (level * LEVEL_BITS);
                        entry.paddr = basePte.ppn;
                        entry.vaddr &= ~((1 << entry.logBytes) - 1);
                        entry.pte = basePte;
                        // put it non-writable into the TLB to detect
                        // writes and redo the page table walk in order
                        // to update the dirty flag.
                        if (!pte.d && mode != BaseMMU::Write)
                            entry.pte.w = 0;
                        doTLBInsert = true;

                        // Update statistics for completed page walks
                        if (level == 1) {
                            walker->pagewalkerstats.num_2mb_walks++;
                        }
                        if (level == 0) {
                            walker->pagewalkerstats.num_4kb_walks++;
                        }
                        DPRINTF(PageTableWalker,
                                "#1 leaf node at level %d, with vpn %#x\n",
                                level, entry.vaddr);
                    }
                }
            } else {
                level--;
                if (level < 0) {
                    DPRINTF(PageTableWalker, "No leaf PTE found,"
                                                  "raising PF\n");
                    doEndWalk = true;
                    fault = pageFault(true);
                } else {
                    Addr shift = (PageShift + LEVEL_BITS * level);
                    Addr idx = (entry.vaddr >> shift) & LEVEL_MASK;
                    nextRead = (pte.ppn << PageShift) + (idx * sizeof(pte));
                    nextState = Translate;
                }
            }
        }
    } else {
        doEndWalk = true;
    }
    PacketPtr oldRead = read;
    Request::Flags flags = oldRead->req->getFlags();

    if (doEndWalk) {
        // If we need to write, adjust the read packet to write the modified
        // value back to memory.
        if (!functional && doWrite) {
            DPRINTF(PageTableWalker, "Writing level%d PTE to %#x: %#x\n",
                level, oldRead->getAddr(), pte);
            write = oldRead;
            write->setLE<uint64_t>(pte);
            write->cmd = MemCmd::WriteReq;
            read = NULL;
        } else {
            write = NULL;
        }

        if (doTLBInsert) {
            if (!functional) {
                
                
                // Fill the entry! 

                Addr vpn = getVPNFromVAddr(entry.vaddr, satp.mode);
                walker->tlb->insert(vpn, entry);
            } else {
                DPRINTF(PageTableWalker, "Translated %#x -> %#x\n",
                        entry.vaddr, entry.paddr << PageShift |
                        (entry.vaddr & mask(entry.logBytes)));
            }
        }
        endWalk();
    }
    else {
        //If we didn't return, we're setting up another read.
        Addr nextReadOffst = nextRead % 64;
        nextRead -= nextReadOffst;
        Addr nextReadIdx = nextReadOffst / 8;
        RequestPtr request = std::make_shared<Request>(
            nextRead, oldRead->getSize(), flags, walker->requestorId, oldRead->req->getPayloadSize(), nextReadIdx);

        delete oldRead;
        oldRead = nullptr;

        read = new Packet(request, MemCmd::ReadReq);
        read->allocate();

        DPRINTF(PageTableWalker,
                "Loading level%d PTE from %#x\nTEST: %#x\n", level, nextRead, read->getSize());
    }

    return fault;
}

void
Walker::WalkerState::endWalk()
{
    nextState = Ready;
    delete read;
    read = NULL;
}

void
Walker::WalkerState::setupWalk(Addr vaddr)
{
    vaddr = Addr(sext<VADDR_BITS>(vaddr));

    Addr shift = PageShift + LEVEL_BITS * 2;
    Addr idx = (vaddr >> shift) & LEVEL_MASK;
    Addr topAddr = (satp.ppn << PageShift) + (idx * sizeof(PTESv39));
    level = 2;

    DPRINTF(PageTableWalker, "Performing table walk for address %#x\n", vaddr);
    DPRINTF(PageTableWalker, "Loading level%d PTE from %#x\n", level, topAddr);
    DPRINTF(PageTableWalker, "The table address is: %#x level%d idx: %#X\n", satp.ppn << PageShift, level, idx);

    Addr offset = topAddr % 64;
    Addr readStartAddr = topAddr - offset;
    Addr offsetIdx = offset >> 3;
    state = Translate;
    nextState = Ready;
    entry.vaddr = vaddr;
    entry.asid = satp.asid;

    Request::Flags flags = Request::PHYSICAL;
    RequestPtr request = std::make_shared<Request>(
        readStartAddr, sizeof(PTESv39) * 8, flags, walker->requestorId, sizeof(PTESv39), offsetIdx);

    read = new Packet(request, MemCmd::ReadReq);
    read->allocate();
    DPRINTF(PageTableWalker, "Test allocation size: %#x\n", read->getSize());
}

bool
Walker::WalkerState::recvPacket(PacketPtr pkt)
{
    assert(pkt->isResponse());
    assert(inflight);
    assert(state == Waiting);
    inflight--;
    if (squashed) {
        // if were were squashed, return true once inflight is zero and
        // this WalkerState will be freed there.
        return (inflight == 0);
    }
    if (pkt->isRead()) {
        // should not have a pending read it we also had one outstanding
        assert(!read);

        // @todo someone should pay for this
        pkt->headerDelay = pkt->payloadDelay = 0;

        state = nextState;
        nextState = Ready;
        PacketPtr write = NULL;
        read = pkt;
        timingFault = stepWalk(write);
        state = Waiting;
        assert(timingFault == NoFault || read == NULL);
        if (write) {
            writes.push_back(write);
        }
        sendPackets();
    } else {
        delete pkt;

        sendPackets();
    }
    if (inflight == 0 && read == NULL && writes.size() == 0) {
        state = Ready;
        nextState = Waiting;
        if (timingFault == NoFault) {
            /*
             * Finish the translation. Now that we know the right entry is
             * in the TLB, this should work with no memory accesses.
             * There could be new faults unrelated to the table walk like
             * permissions violations, so we'll need the return value as
             * well.
             */
            Addr vaddr = req->getVaddr();
            vaddr = Addr(sext<VADDR_BITS>(vaddr));
            Addr paddr = walker->tlb->hiddenTranslateWithTLB(vaddr, satp.asid,
                                                             satp.mode, mode);
            req->setPaddr(paddr);

            // do pmp check if any checking condition is met.
            // timingFault will be NoFault if pmp checks are
            // passed, otherwise an address fault will be returned.
            timingFault = walker->pmp->pmpCheck(req, mode, pmode, tc);

            if (timingFault == NoFault) {
                timingFault = walker->pma->check(req, mode);
            }

            // Let the CPU continue.
            translation->finish(timingFault, req, tc, mode);
        } else {
            // There was a fault during the walk. Let the CPU know.
            translation->finish(timingFault, req, tc, mode);
        }
        return true;
    }
    return false;
}

void
Walker::WalkerState::sendPackets()
{
    //If we're already waiting for the port to become available, just return.
    if (retrying)
        return;

    //Reads always have priority
    if (read) {
        PacketPtr pkt = read;
        read = NULL;
        inflight++;
        if (!walker->sendTiming(this, pkt)) {
            retrying = true;
            read = pkt;
            inflight--;
            return;
        }
    }
    //Send off as many of the writes as we can.
    while (writes.size()) {
        PacketPtr write = writes.back();
        writes.pop_back();
        inflight++;
        if (!walker->sendTiming(this, write)) {
            retrying = true;
            writes.push_back(write);
            inflight--;
            return;
        }
    }
}

unsigned
Walker::WalkerState::numInflight() const
{
    return inflight;
}

bool
Walker::WalkerState::isRetrying()
{
    return retrying;
}

bool
Walker::WalkerState::isTiming()
{
    return timing;
}

bool
Walker::WalkerState::wasStarted()
{
    return started;
}

void
Walker::WalkerState::squash()
{
    squashed = true;
}

void
Walker::WalkerState::retry()
{
    retrying = false;
    sendPackets();
}

Fault
Walker::WalkerState::pageFault(bool present)
{
    DPRINTF(PageTableWalker, "Raising page fault.\n");
    return walker->tlb->createPagefault(entry.vaddr, mode);
}

Walker::PagewalkerStats::PagewalkerStats(statistics::Group *parent)
  : statistics::Group(parent),
    ADD_STAT(num_4kb_walks, statistics::units::Count::get(),
             "Completed page walks with 4KB pages"),
    ADD_STAT(num_2mb_walks, statistics::units::Count::get(),
             "Completed page walks with 2MB pages")
{
}

} // namespace RiscvISA
} // namespace gem5
