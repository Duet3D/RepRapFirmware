import sys
import time

dsu_address = 0x41002000
ctrl_reg_offset = 0x100
statusa_offset = 0x101
statusa_offset_internal = 0x001
ctrl_ce_bit = 0x10
statusa_done = 0x1
statusa_crstext = 0x2
erase_size = 0x2000

# information from NvmControllerInformation in atbackend
class flash_info:
    moduleName = "NVMCTRL"
    mappedAddress = 0x0000000000000000
    size = 0x00100000
    pageSize = 0x00000200
    hasUserpage = 1
    mappedUserPageAddress = 0x0000000000804000
    userPageSize = 0x00000010
    nPagesPerLockRegion = 0xcdcdcdcd
    pageSizeParameter = 0xcdcdcdcd
    flashSizeParameter = 0xcdcdcdcd
    lockbitsSizeInBytes = 0xcdcdcdcd
    hasRWW = 0
    mappedRwwAddress = 0x0000000000000000
    rWWSectionOffsetFromFlashStart = 0x0000000000000000
    rwwSectionSize = 0x0000000000000000
    baseAddress = 0x0000000041004000  # Address of the flash controller module
    fcrRegisterAddress = 0xcdcdcdcdcdcdcdcd # Address of the Flash Control register , fmr for eeflashcontroller
    fcmdRegisterAddress = 0xcdcdcdcdcdcdcdcd # Address of the flash command register
    fsrRegisterAddress = 0xcdcdcdcdcdcdcdcd # Address of the Flash status register
    frrRegisterAddress = 0xcdcdcdcdcdcdcdcd #Result register address. only for eeflashcontroller
    userPageOffsetFromFlashStart = 0x0000000000804000
    fuseGpHiAddress = 0xcdcdcdcdcdcdcdcd
    fuseGpLoAddress = 0xcdcdcdcdcdcdcdcd # used for gpnvm for eeflashcontroller
    fuseGpMasks = 0x000000ff # Encode up to 32 mask bits for GPNVMbits
    mSectorSize = 0x0010
    mSizeEwpSector = 0x0000
    nvmCtrl2409_E54 = 1
    nvmCtrl2802_L1x = 0
    cmdRegOfs = 0x00000004
    cfgRegOfs = 0x00000000
    cmdRegSize = 0x00000002
    cfgRegSize = 0x00000002
    stsRegSize = 0x00000002
    intRegSize = 0x00000002
    stsRegOfs = 0x12
    intFlagOfs = 0x10

# DHCSR bitfields
DBGKEY       = 0xa05f0000
S_RESET_ST   = 0x02000000
S_RETIRE_ST  = 0x01000000
S_SDE        = 0x00100000
S_LOCKUP     = 0x00080000
S_SLEEP      = 0x00040000
S_HALT       = 0x00020000
S_REGRDY     = 0x00010000
C_SNAPSTALL  = 0x00000020
C_MASKINTS   = 0x00000008
C_STEP       = 0x00000004
C_HALT       = 0x00000002
C_DEBUGEN    = 0x00000001

# DEMCR bitfields
TRCENA       = 0x01000000
MON_REQ      = 0x00080000
MON_STEP     = 0x00040000
MON_PEND     = 0x00020000
MON_EN       = 0x00010000
VC_HARDERR   = 0x00000400
VC_INTERR    = 0x00000200
VC_BUSERR    = 0x00000100
VC_STATERR   = 0x00000080
VC_CHKERR    = 0x00000040
VC_NOCPERR   = 0x00000020
VC_MMERR     = 0x00000010
VC_CORERESET = 0x00000001

# AIRCR bitfields
VECTKEY      = 0x05fa0000
SYSRESETREQ  = 0x00000004
VECTRESET    = 0x00000001

# NVIC used for getting ARM core type
NVIC_Addr = 0xE000E000
NVIC_CPUID = NVIC_Addr + 0x0D00
CPUID_PARTNO = 0x0000FFF0  # Part Number Mask

NVMCTRL_USER = 0x00804000  # (NVMCTRL) USER Base Address
CMD_AUX_ERASE_PAGE = 0xA500 # NVMCtrl erase user page
CMD_WRITE_QUAD_WORD = 0xA504    # write quad word command (flash or user page)

isConnected = False  # are we connected to the device?

# for getting BP info
FPB_CTRL = 0xE0002000
FPB_REV_P = 28 # Revision Position
FPB_NUM_CODE = 0x000000F0 # Number of Code Slots Mask
FPB_NUM_CODE_P = 4 # Number of Code Slots Position

def begin_communication_session():
    dap.LogPackets(0)
    log.getLogLevelThreshold()
    log.setShowOutput(False)
    log.info("Cortex-M4 NVMCTRL2 programmer/debugger")

def printDHCSR(text):
    d = dap.ReadD32(dap.DHCSR)
    decodeAndPrintDHCSR(text, d)

def decodeAndPrintDHCSR(text, d):
    strDHCSR=" %s %x : " % (text,d)
    if (d & S_RESET_ST):
        strDHCSR = strDHCSR +  "S_RESET_ST."
    if (d & S_RETIRE_ST):
        strDHCSR = strDHCSR +  "S_RETIRE."
    if (d & S_SDE):
        strDHCSR = strDHCSR +  "S_SDE."
    if (d & S_LOCKUP):
        strDHCSR = strDHCSR +  "S_LOCKUP."
    if (d & S_SLEEP):
        strDHCSR = strDHCSR +  "S_SLEEP."
    if (d & S_HALT):
        strDHCSR = strDHCSR +  "S_HALT."
    if (d & S_REGRDY):
        strDHCSR = strDHCSR +  "S_REGRDY."
    if (d & C_SNAPSTALL):
        strDHCSR = strDHCSR +  "C_SNAPSTALL."
    if (d & C_MASKINTS):
        strDHCSR = strDHCSR +  "C_MASKINTS."
    if (d & C_STEP):
        strDHCSR = strDHCSR +  "C_STEP."
    if (d & C_HALT):
        strDHCSR = strDHCSR +  "C_HALT."
    if (d & C_DEBUGEN):
        strDHCSR = strDHCSR +  "C_DEBUGEN"
    log.info(strDHCSR)

#import program_api
#reload (debug_api)
# this file contains all the methods that satisfy the APIs exposed by ScriptBasedController
# RI4ProgramInterface

def blank_check():
    log.info("RI4ProgramInterface: Blank check")

def get_arm_type():
    val = dap.ReadD32(NVIC_CPUID)
    switcherCPUPartno = {
        0xC330: "ARM_SC300",
        0xC300: "ARM_SC000",
        0xC240: "ARM_CM4",
        0xC230: "ARM_CM3",
        0xC210: "ARM_CM1",
        0xC200: "ARM_CM0",
        0xC600: "ARM_CM0P", # Flycatcher
        0xC270: "ARM_CM7",   # Pelican
        0xD200: "ARM_CM8"   # L10
    }
    return switcherCPUPartno.get(val & CPUID_PARTNO, "Unknown core...")

def enable_bp():
    fpb_ctrl = dap.ReadD32(FPB_CTRL)
    fpb_rev = fpb_ctrl >> FPB_REV_P
    num_hwbp = (fpb_ctrl & FPB_NUM_CODE) >> FPB_NUM_CODE_P
    log.info("FPB ref = %X, #HW BP's = %X" % (fpb_rev, num_hwbp))
    dap.WriteD32(FPB_CTRL, fpb_ctrl | 3)

has_entered_reset_extension = False

def resetExtension():
    global has_entered_reset_extension
    has_entered_reset_extension = True
    timeout=1000
    log.info("Reset extension")
    dap.Pins(0,dap.RESET,timeout)  # 1 msec with reset high
    dap.Pins(dap.RESET,dap.RESET,timeout) # 1 msec with floating reset
    dap.Pins(0,dap.RESET | dap.SWCLK,timeout); # 1 msec with reset and clock low
    dap.Pins(dap.RESET,dap.RESET,timeout); # now float reset back

def isNvmReady():
    status = dap.ReadD16(flash_info.baseAddress + flash_info.stsRegOfs)
    isReady = 0 != (status & 1)
    if not isReady:
        return False

    intFlag = dap.ReadD16(flash_info.baseAddress + flash_info.intFlagOfs)
    isError = 0 != (intFlag & (2 | 4 | 8 | 16 | 32 | 64)) # Check ERROR flags
    if not isError:
        return True

    clearNvmErrorBits()
    raise "NVMctrl reports an error"

def clearNvmErrorBits():
    # clear error bits in STATUS and INTFLAG registers
    status = dap.ReadD16(flash_info.baseAddress + flash_info.stsRegOfs)
    intFlag = dap.ReadD16(flash_info.baseAddress + flash_info.intFlagOfs)
    log.error("NVMCtrl error bits set! (STATUS = 0x%X, INTFLAG = 0x%X)" % (status, intFlag))

    # clear error bits by setting 1
    dap.WriteD16(flash_info.baseAddress + flash_info.intFlagOfs, intFlag | (2 | 4 | 8 | 16 | 32 | 64)) 

def erase():
    log.info("RI4ProgramInterface: Erase")
    resetExtension()

    dap.Connect(True,2000000L)
    global isConnected
    isConnected = True

    dap.WriteD8(dsu_address + ctrl_reg_offset, ctrl_ce_bit) # DSU CTRL.CE
    log.info("Issued DSU.CE STATUSA now %x"  % dap.ReadD8(dsu_address + statusa_offset))

    n = 0
    while n<100: # wait for erase finished when STATUSA DONE is set
        statusa=dap.ReadD8(dsu_address + statusa_offset) # 0x41002101
        log.info("Read DSU.STATUSA %x" % statusa)
        if statusa & statusa_done: # chip erase finished?
            break
        time.sleep(max(1-n*0.12,0.1))
        n=n+1

    issueResetAndWait()

    enable_bp()
    log.info("ARM core: " + get_arm_type())

def issueResetAndWait():
    log.info("RI4ProgramInterface: issueResetAndWait")
    dap.WriteD32(dap.DEMCR, TRCENA|VC_CORERESET) #0x01000001
    dap.WriteD32(dap.AIRCR, VECTKEY|SYSRESETREQ) # 1=VECTRESET 4=SYSRESETREQ 0x05fa0004

    n = 0
    retries = 100
    seenReset = False
    while n<retries: # wait for reset and halt
        dhcsr=dap.ReadD32(dap.DHCSR)
        decodeAndPrintDHCSR("dap.DHCSR",dhcsr)
        if (dhcsr & S_RESET_ST): # wait for reset
            seenReset=True
            log.info("Set HALT and CRSTEXT to release from reset extension")
            dap.WriteD32(dap.DHCSR, DBGKEY|C_HALT|C_DEBUGEN) # 0xa05f0003
            global has_entered_reset_extension
            statusa = dap.ReadD8(dsu_address + statusa_offset_internal)
            if not (statusa & statusa_crstext):
                if has_entered_reset_extension:
                    log.error("ERROR: Device failed to enter reset extension. Check cap on RST line.")
            has_entered_reset_extension = False
            dap.WriteD8(dsu_address + statusa_offset_internal, statusa_crstext) # release the cpu 0x41002101,0x2
            printDHCSR("After clearing CRSTEXT")
        hasHalted = 0 != (dhcsr & S_HALT) # wait for S_HALT
        if seenReset:
            if hasHalted:
                log.info("Have seen reset and now halted")
                break
        time.sleep(0.1)
        n=n+1

    dap.WriteD32(dap.DEMCR, TRCENA)
    testRegs()
    if n==retries:
        raise "timeout in reset"

def hold_in_reset():
    log.info("RI4ProgramInterface: Hold in reset")
    global isConnected
    if not isConnected:
      dap.Connect(True, 2000000L)
      isConnected = True
    issueResetAndWait()

def release_from_reset():
    log.info("RI4ProgramInterface: Release from reset")

    # toggle reset line
    dap.Pins(0,dap.RESET,1000)
    dap.Pins(dap.RESET,dap.RESET,1000) # now float reset back

    # workaround if reset line is not connected from debugger to target
    #dap.WriteD32(dap.AIRCR, VECTKEY|SYSRESETREQ) # 1=VECTRESET 4=SYSRESETREQ 0x05fa0004

def write_reg(adr, val, size):
    if size == 4:
        dap.WriteD32(adr, val)
    elif size == 2:
        dap.WriteD16(adr, val)
    elif size == 1:
        dap.WriteD8(adr, val)
    else:
        log.info("Error write_reg size == %i" % size)

def read_reg(adr, size):
    if size == 4:
        return dap.ReadD32(adr)
    elif size == 2:
        return dap.ReadD16(adr)
    elif size == 1:
        return dap.ReadD8(adr)
    else:
        log.info("Error read_reg size == %i" % size)
        return 0x0

# global variable to store config register value before programming
prevCfgReg = 0x0

def save_cfg_word(user_page = False):
    cfgAdr = flash_info.baseAddress + flash_info.cfgRegOfs
    cfgSize = flash_info.cfgRegSize
    global prevCfgReg
    prevCfgReg = read_reg(cfgAdr, cfgSize)

    newCfgReg = (prevCfgReg & ~4) | 0x0f00;  # set WS 15 #alternative: 4; # set AUTOWS
    if user_page:
        newCfgReg = newCfgReg | 0x00  # set manual write
    else:
        newCfgReg = newCfgReg | 0x30 # set auto page write

    if newCfgReg != prevCfgReg:
        log.info(
            "save_cfg_word: Write prevCfgReg = %x, newCfgReg = %x (user_page = %s)" %
            (prevCfgReg, newCfgReg, "True" if user_page else "False"))
        write_reg(cfgAdr, newCfgReg, cfgSize)
    else:
        log.info(
            "save_cfg_word: prevCfgReg = %x unchanged, skipped saving (user_page = %s)" %
            (prevCfgReg, "True" if user_page else "False"))

def reset_cfg_word():
    log.info("reset_cfg_word: prevCfgReg = %x" % prevCfgReg)
    cfgAdr = flash_info.baseAddress + flash_info.cfgRegOfs
    cfgSize = flash_info.cfgRegSize
    write_reg(cfgAdr, prevCfgReg, cfgSize)

def eraseUserPage(addr):
    log.info("Erase row at address %X" % addr)
    status = dap.ReadD16(flash_info.baseAddress + flash_info.stsRegOfs)
    intFlag = dap.ReadD8(flash_info.baseAddress + flash_info.intFlagOfs)
    log.info("eraseUserPage: NVMCtrl STATUS = 0x%X, INTFLAG = 0x%X" % (status, intFlag))

    dap.WriteD32(addr, 0xFFFFFFFF) # write a few bytes to the page buffer to set address register
    writeCommand (CMD_AUX_ERASE_PAGE, 20)

def writeCommand(command, t):
    dap.WriteD16(flash_info.baseAddress + flash_info.cmdRegOfs, command) # 0x41004004
    while t > 0 :
        if isNvmReady():
            return
        time.sleep(0.001)
        t  = t - 1
    if t == 0:
        raise "timeout executing NVM command"

def writePageManualQW(addr, ofs, data):
    log.info("Write row at address %X, offset %X" % (addr, ofs))
    status = dap.ReadD16(flash_info.baseAddress + flash_info.stsRegOfs)
    intFlag = dap.ReadD8(flash_info.baseAddress + flash_info.intFlagOfs)
    log.info("writeRow: NVMCtrl STATUS = 0x%X, INTFLAG = 0x%X" % (status, intFlag))

    dap.WriteD32(addr, 0xFFFFFFFF) # write a few bytes to the page buffer to set address register
    erase_size_user_row = flash_info.pageSize
    quad_word_bytes = 16
    written = 0
    while written < erase_size_user_row:
        log.info("writeRow call dap.WriteMem address = 0x%x-+0x%x" % (addr, addr + quad_word_bytes-1))
        dap.WriteMem(addr, data, ofs + written, quad_word_bytes)
        writeCommand(CMD_WRITE_QUAD_WORD, 20)
        addr += quad_word_bytes
        written += quad_word_bytes

def writeCfgPage(address, data, nBytes):
    if nBytes != 512:
        log.error("writeCfgPage called with %X bytes data, only handle 0x200" % nBytes)
    pageCopy = bytearray(data)

    # Make sure the device is halted
    resetExtension()

    dap.Connect(True,2000000L) # connect after reset
    global isConnected
    isConnected = True
    dhcsr = dap.ReadD32(dap.DHCSR)
    decodeAndPrintDHCSR("dap.DHCSR",dhcsr)
    issueResetAndWait()

    if address == NVMCTRL_USER:
        eraseUserPage(address) # Erase page 512 bytes
        writePageManualQW(address, 0, pageCopy)
    else:
        log.error("Trying to writeCfgPage but address is not %X but %X" % (NVMCTRL_USER, address))

def write_flash_page(adr, ofs, data, length):
    dap.WriteD32(adr, 0)
    #dap.WriteD16(flash_info.baseAddress, 0xa502)
    log.info("RI4ProgramInterface: write_flash_page adr %x, length %X" % (adr, length))

    page_size = flash_info.pageSize
    written = 0
    while written < erase_size and written < length:
        log.info("write_flash_page call dap.WriteMem address = 0x%x-0x%x" % (adr, adr + page_size-1))
        dap.WriteMem(adr, data, ofs + written, page_size)
        adr += page_size
        written += page_size

def isUserPageStart(address, length):
    return flash_info.hasUserpage and address == flash_info.userPageOffsetFromFlashStart

def prog_write(type_of_mem, address, length, data):
    log.info("RI4ProgramInterface: Writing %d bytes to address 0x%0x of %s memory" % (length, address, type_of_mem))
    orig_address = address

    global isConnected
    if not isConnected:
        dap.Connect(True,2000000L)
        isConnected = True

    if address == NVMCTRL_USER: # user page / fuses / configuration bits
        if length > 512:
            log.error("prog_write Cfg, length (0x%X) > 0x200 bytes, only handle writing 0x200 bytes for user page!" % length)
        save_cfg_word(True)
        writeCfgPage(address, data, length)
    else:
        save_cfg_word(False)
        written = 0
        while written < length:
            log.info("RI4ProgramInterface: Write to address 0x%0x offset 0x%0x of %s memory" % (orig_address, written, type_of_mem))
            write_flash_page(address,written,data, min(length - written, erase_size))
            written += erase_size
            address += erase_size

    reset_cfg_word()

def prog_read(type_of_mem, address, length, data):
    log.info("RI4ProgramInterface: Reading %d bytes from address 0x%0x of %s memory" % (length, address, type_of_mem))

    global isConnected
    if not isConnected:
        dap.Connect(True,2000000L)
        isConnected = True

    dap.ReadMem(address,data,0,length)

def verify_transfer(type_of_mem, address, data, length):
    log.info("RI4ProgramInterface: Verifying %d bytes to address 0x%0x of %s memory" % (length, address, type_of_mem))

def end_of_operations():
    log.info("RI4ProgramInterface: End of operations")
    #issueResetAndWait()
    dap.Disconnect()
    global isConnected
    isConnected = False

# this file contains all the methods that satisfy the APIs exposed by ScriptBasedController
# RI4DebugInterfaceWithBP

# global variables to keep track of debug mode and running status
global g_in_debug_mode
g_in_debug_mode = False

global g_is_running 
g_is_running = False

def begin_debug_session():
    log.info("Debug:: Init debug session")
    global g_in_debug_mode
    global g_is_running
    global isConnected
    if not isConnected:
      dap.Connect(True, 2000000L)
      isConnected = True
    issueResetAndWait()
    testRegs()
    g_in_debug_mode = True
    g_is_running = False

def debug_read(mem_type, start, length, data):
    log.info("Debug:: Reading %d bytes at start address 0x%0x (%s)" %
             (length, start, mem_type))
    dap.ReadMem(start, data, 0, length)


def debug_write(mem_type, start, length, data):
    log.info("Debug:: Writing %d bytes at start address 0x%0x (%s)" %
             (length, start, mem_type))
    dap.WriteMem(start, data, 0, length)

def is_target_in_debug_mode():
    log.info("Debug:: Is target in debug mode? %s" % g_in_debug_mode)
    return g_in_debug_mode

def get_pc():
    rv = dap.GetARMReg(13)
    pc = dap.GetARMReg(15)
    log.info("get_pc SP=%x PC=%x" %  (rv,pc))
    return pc

def run_target():
    global g_is_running
    g_is_running = True
    printDHCSR("Target to be set to running ")
    dap.WriteD32(dap.DHCSR, DBGKEY|C_DEBUGEN) # 0xa05f0001 (C_HALT == 0)
    printDHCSR("Target should be running ")

def halt_target():
    global g_is_running
    g_is_running = False
    printDHCSR("Target to be halted ")
    dap.WriteD32(dap.DHCSR, DBGKEY|C_DEBUGEN|C_HALT) # halt 0xA05F0003
    printDHCSR("Target was halted ")

def step_target():
    printDHCSR("Debug:: step target")
    get_pc()
    dap.WriteD32(dap.DHCSR, DBGKEY|C_MASKINTS|C_HALT|C_DEBUGEN)  # 0xa05f000b
    dap.WriteD32(dap.DHCSR, DBGKEY|C_MASKINTS|C_STEP|C_DEBUGEN)  # step 0xa05f000d
    dap.WriteD32(dap.DHCSR, DBGKEY|C_HALT|C_DEBUGEN) # halt 0xa05f0003
    get_pc()

def set_pc(pc):
    log.info("Debug:: set pc to 0x%0x" % pc)
    dap.SetARMReg(15,pc)
    get_pc()

def print_byte_array(text, array, index, size):
    string = text
    for i in range(size):
        string += " %x" % array[i+index]
    log.info(string)

def set_sw_bp(address, instruction, flags):
    log.info("Debug:: set/reset pb at address 0x%0x, store instructions 0x%0x, flags = 0x%0x" % (address, instruction, flags))
    erase_page_start = address & ~(erase_size-1)
    address_in_erase_page = address & erase_size-1
    log.info("set_sw_bp: erase_page_start = %x, address_in_erase_page = %x" % (erase_page_start, address_in_erase_page))

    data = bytearray(256)
    dap.ReadMem(erase_page_start, data, 0, erase_size)
    print_byte_array("set_sw_bp: data[address_in_erase_page] =", data, address_in_erase_page, 4)
    return_value = data[address_in_erase_page] | (data[address_in_erase_page+1] << 8)
    log.info("set_sw_bp: return_value (replaced instruction) = %x" % return_value)

    # Insert software breakpoint instruction in the data
    data[address_in_erase_page] = instruction & 0xFF
    data[address_in_erase_page+1] = (instruction >> 8) & 0xFF

    print_byte_array("set_sw_bp: data[address_in_erase_page] after changing instr =", data, address_in_erase_page, 4)
    prog_write("Pgm", erase_page_start, erase_size, data)

    return return_value

def reset_target():
    global regsAreInvalid
    log.info("Debug:: Reset target")
    regsAreInvalid = True
    issueResetAndWait()

def is_target_running():
    global g_is_running, regsAreInvalid
    dhcsr = dap.ReadD32(dap.DHCSR)
    state =  0 == (dhcsr & S_HALT)
    if state != g_is_running:
        log.info("Debug:: is_running %s" % state)
        regsAreInvalid = True
        g_is_running = state
    return g_is_running

def is_running2():
    dhcsr = dap.ReadD32(dap.DHCSR)
    return (0 == (dhcsr & S_HALT))

def end_debug_session():
    global g_in_debug_mode
    global g_is_running
    g_is_running = False
    g_in_debug_mode = False

def testRegs():
    get_pc()
    return
 
