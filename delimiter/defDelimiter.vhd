library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library mylib;
use mylib.defHeartBeatUnit.all;
use mylib.defDataBusAbst.all;

package defDelimiter is
  -- Data type -----------------------------------------------------------------------------
  constant kWidthDataType             : integer  := 6;

  constant kDatatypeTDCData           : std_logic_vector (kWidthDataType-1 downto 0) := "001011"; -- TDC leading ata
  constant kDatatypeTDCDataT          : std_logic_vector (kWidthDataType-1 downto 0) := "001101"; -- TDC trailing ata

  constant kDatatypeCAdcData          : std_logic_vector (kWidthDataType-1 downto 0) := "001100"; -- Corrected ADC data
  constant kDatatypeAdcHead           : std_logic_vector (kWidthDataType-1 downto 0) := "001110"; -- ADC sub-frame header
  constant kDatatypeAdcTrail          : std_logic_vector (kWidthDataType-1 downto 0) := "001010"; -- ADC sub-frame trailer
  constant kDatatypeAdcData           : std_logic_vector (kWidthDataType-1 downto 0) := "001000"; -- ADC sub-frame payload

  constant kDataTypeIThrottleT1Start  : std_logic_vector (kWidthDataType-1 downto 0) := "011001"; -- Input Throttling Type2 Start timing
  constant kDataTypeIThrottleT1End    : std_logic_vector (kWidthDataType-1 downto 0) := "010001"; -- Input Throttling Type2 End timing
  constant kDataTypeIThrottleT2Start  : std_logic_vector (kWidthDataType-1 downto 0) := "011010"; -- Input Throttling Type2 Start timing
  constant kDataTypeIThrottleT2End    : std_logic_vector (kWidthDataType-1 downto 0) := "010010"; -- Input Throttling Type2 End timing

  constant kDatatypeHeartbeat         : std_logic_vector (kWidthDataType-1 downto 0) := "011100"; -- 1st heartbeat delimiter
  constant kDatatypeHeartbeatT2       : std_logic_vector (kWidthDataType-1 downto 0) := "011110"; -- 2nd heartbeat delimiter

  -- Delimiter flag definition ---------------------------------------------------------------
  constant kWidthDelimiterFlag    : integer  := 16;  -- Delimiter flag width

  constant kIndexSOS              : integer  := 15;  -- 16th, SOS signal to software
  constant kIndexRadiationURE     : integer  := 14;  -- 15th, Uncorrectable error happens by radiation
  constant kIndexMikumariError    : integer  := 13;  -- 14th, Some communication error happens in MIKUMARI-link
  --constnat kReserve             : integer  := 12;  -- 13th,
  constant kIndexOverflow         : integer  := 11;  -- 12th, IncomingFIFO almost-full or full flags goes high
  constant kIndexGHbfNumMismatch  : integer  := 10;  -- 11th,  Stcp HbfNumber and local HbfNumber mismatch
  constant kIndexLHbfNumMismatch  : integer  := 9;   -- 10th,  Hbf numbers mismatch among merger units
  --constnat kReserve             : integer  := 8;   -- 9th,
  constant kIndexInThrottlingT1   : integer  := 7;  -- 8th,  Input throttling type1 is working
  constant kIndexInThrottlingT2   : integer  := 6;  -- 7th,  Input throttling type2 is working
  constant kIndexOutThrottling    : integer  := 5;  -- 6th,  Output throttling is working
  constant kIndexHbfThrottling    : integer  := 4;  -- 5th,  Hbf throttlinng is working
  --constnat kReserve             : integer  := 3;  -- 4th,
  --constnat kReserve             : integer  := 2;  -- 3th,
  constant kIndexFrameFlag2       : integer  := 1;  -- 2nd,
  constant kIndexFrameFlag1       : integer  := 0;  -- 1st,

  function genFlagVector(index : integer; flag : std_logic) return std_logic_vector;

  -- Heartbeat frame, counter ----------------------------------------------------------------
  constant kWidthStrHbf       : integer  := 24;  -- Heartbeat frame number
  constant kWidthStrHbc       : integer  := 16;  -- Heartbeat counter value

  constant kMaxHBFrame        : std_logic_vector (kWidthStrHbf-1 downto 0) := X"FFFFFF"; -- heartbeat max frame number
  constant kMaxHBCount        : std_logic_vector (kWidthStrHbc-1 downto 0) := X"FFFF";   -- heartbeat max count number

  -- Data bit position ---------------------------------------------------------------------
  constant kMSBDataType     : integer  := kWidthData-1;
  constant kLSBDataType     : integer  := kMSBDataType-kWidthDataType+1;

  -- Delimiter data --
  -- 1st delimiter: 6-bit data-type, 2-bit reserve, 16-bit flag,     16-bit Time offset,         24-bit HBF number
  -- 2nd delimiter: 6-bit data-type, 2-bit reserve, 16-bit user reg, 20-bit generated ddta size, 20-bit transferred data size

  -- 1st delimiter --
  constant kPosHbdDataType  : std_logic_vector(kWidthData-1          downto kWidthData          -kWidthDataType)      := (others => '0');
  constant kPosHbdReserve1  : std_logic_vector(kPosHbdDataType'low-1 downto kPosHbdDataType'low -2)                   := (others => '0');
  constant kPosHbdFlag      : std_logic_vector(kPosHbdReserve1'low-1 downto kPosHbdReserve1'low -kWidthDelimiterFlag) := (others => '0');
  constant kPosHbdOffset    : std_logic_vector(kPosHbdFlag'low-1     downto kPosHbdFlag'low     -16)                  := (others => '0');
  constant kPosHbdHBFrame   : std_logic_vector(kPosHbdOffset'low-1   downto kPosHbdOffset'low   -kWidthStrHbf)        := (others => '0');

  -- 2nd delimiter --
  constant kPosHbdUserReg   : std_logic_vector(kPosHbdReserve1'low-1 downto kPosHbdReserve1'low -16):= (others => '0');
  constant kPosHbdGenSize   : std_logic_vector(kPosHbdUserReg'low-1 downto kPosHbdUserReg'low   -20):= (others => '0');
  constant kPosHbdTransSize : std_logic_vector(kPosHbdGenSize'low-1  downto kPosHbdGenSize'low  -20):= (others => '0');

  function checkDelimiter(data_type : std_logic_vector) return boolean;
  function checkUsrData(data_type : std_logic_vector) return boolean;

  end package defDelimiter;
-- ----------------------------------------------------------------------------------
-- Package body
-- ----------------------------------------------------------------------------------
package body defDelimiter is
  function genFlagVector(index : integer; flag : std_logic) return std_logic_vector is
    variable result   : std_logic_vector(kWidthDelimiterFlag-1 downto 0) := (others => '0');
  begin
    result(index)  := flag;
    return result;
  end genFlagVector;

  function checkDelimiter(data_type : std_logic_vector) return boolean is
  begin
    if(data_type = kDatatypeHeartbeat or data_type = kDatatypeHeartbeatT2) then
      return true;
    else
      return false;
    end if;
  end checkDelimiter;

  function checkUsrData(data_type : std_logic_vector) return boolean is
  begin
    if(data_type = kDatatypeTDCData or data_type = kDatatypeTDCDataT or data_type = kDatatypeAdcData) then
      return true;
    else
      return false;
    end if;
  end checkUsrData;

end package body defDelimiter;
