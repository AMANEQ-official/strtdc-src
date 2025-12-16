library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_std.all;
use IEEE.STD_LOGIC_MISC.ALL;

Library xpm;
use xpm.vcomponents.all;

library mylib;
use mylib.defHrTimingUnit.all;
use mylib.defFineCountLUT.all;
use mylib.defDataBusAbst.all;
use mylib.defDelimiter.all;
use mylib.defTDC.all;
use mylib.defLACCP.all;


entity ODPBlockNoTot is
  generic(
    kNumInput     : integer := 32;
    enDEBUG       : boolean := false
  );
  port(
    -- System --
    genChOffset     : in std_logic;

    rst             : in std_logic;
    tdcClk          : in std_logic;
    baseClk         : in std_logic;
    hitOut          : out std_logic_vector(kNumInput-1 downto 0);
    --userReg         : in  std_logic_vector(kPosHbdUserReg'length-1 downto 0);

    -- LACCP --
    LaccpFineOffset : in signed(kWidthLaccpFineOffset-1 downto 0);

    -- Control regisers --
    regThrough      : in std_logic;
    regAutoSW       : in std_logic;
    regSwitch       : in std_logic;
    regReadyLut     : out std_logic;
    regEnInv        : in std_logic;

    regTdcMaskL     : in std_logic_vector(kNumInput-1 downto 0);
    regTdcMaskT     : in std_logic_vector(kNumInput-1 downto 0);

    enBypassDelay   : in  std_logic;
    enBypassParing  : in  std_logic;
    enBypassOfsCorr : in  std_logic;

    enTotFilter     : in  std_logic;
    enTotZeroThrough  : in std_logic;
    totMinTh        : in  std_logic_vector(kWidthTOT-1 downto 0);
    totMaxTh        : in  std_logic_vector(kWidthTOT-1 downto 0);

    testModeIn      : in std_logic;

    -- Data flow control --
    daqOn           : in  std_logic;
    hbfThrottlingOn : in  std_logic;
    triggerGate     : in  std_logic;

    -- Heartbeat count for TDC
    hbCount         : in  std_logic_vector(kWidthStrHbc-1 downto 0);

    -- Delimiter
    validDelimiter  : in  std_logic;
    dInDelimiter    : in  std_logic_vector(kWidthIntData-1 downto 0);

    -- Data In --
    sigIn           : in  std_logic_vector(kNumInput-1 downto 0);
    calibIn         : in  std_logic;

    -- Data Out --
    validOut        : out std_logic_vector(2*kNumInput-1 downto 0);
    dOut            : out DataArrayType(2*kNumInput-1 downto 0)

  );
end ODPBlockNoTot;

architecture RTL of ODPBlockNoTot is
  attribute keep : string;
  attribute keep_hierarchy : string;

  -- System --
  signal sync_reset       : std_logic;
  signal sync_reset_tdc   : std_logic_vector(kNumInput-1 downto 0);
  signal daq_off_reset    : std_logic;
  attribute keep of sync_reset_tdc  : signal is "true";

  function GetChOffset(genChOffset : std_logic) return integer is
  begin
    if(genChOffset = '1') then
      return 32;
    else
      return 0;
    end if;
  end function;

  -- Signal decralation ---------------------------------------------
  -- system --
  signal reg_through, reg_auto_sw, reg_switch : std_logic;
  signal reg_ready_l, reg_ready_t             : std_logic;

  signal sig_in_n           : std_logic_vector(kNumInput-1 downto 0);

  -- TDC --
  -- Delay taps -------------------------------------------------------------
  type TapArray   is array (integer range kNumInput-1 downto 0) of std_logic_vector(kNumRTaps downto 0);

  signal tap_out        : tapArray;
  signal leading_taps   : tapArray;
  signal trailing_taps  : tapArray;

  function GetInvVector(is_inversion : std_logic) return std_logic_vector is
    variable  result  : std_logic_vector(kNumRTaps downto 0);
  begin
    if(is_inversion = '1') then
      result  := (others => '1');
    else
      result  := (others => '0');
    end if;

    return result;
  end function;

  type dRawFineType is array (integer range kNumInput-1 downto 0) of std_logic_vector(kWidthFine+kWidthSemi-1 downto 0);

  signal valid_raw_leading      : std_logic_vector(kNumInput -1 downto 0);
  signal raw_fc_leading         : dRawFineType;
  signal valid_raw_trailing     : std_logic_vector(kNumInput -1 downto 0);
  signal raw_fc_trailing        : dRawFineType;

  -- Estimater LUT --
  signal ready_lut_leading      : std_logic_vector(kNumInput-1 downto 0);
  signal ready_lut_trailing     : std_logic_vector(kNumInput-1 downto 0);

  type dFclType is array (integer range kNumInput-1 downto 0) of std_logic_vector(kWidthLutOut-1 downto 0);
  signal hdout_fcl_leading      : std_logic_vector(kNumInput -1 downto 0);
  signal dout_fcl_leading       : dFclType;
  signal hdout_fcl_trailing     : std_logic_vector(kNumInput -1 downto 0);
  signal dout_fcl_trailing      : dFclType;

--  type dSemiType is array (integer range kNumInput-1 downto 0) of std_logic_vector(kWidthSemi-1 downto 0);
--  signal semi_count_leading_1   : dSemiType;
--  signal semi_count_leading_2   : dSemiType;
--  signal semi_count_leading_3   : dSemiType;
--
--  signal semi_count_trailing_1  : dSemiType;
--  signal semi_count_trailing_2  : dSemiType;
--  signal semi_count_trailing_3  : dSemiType;

  -- Generate FineCount /(Estimator+SemiFine) --
  constant fzero                : std_logic_vector(kWidthFineCount-1 downto 0):= (others => '0');
  signal pos_fc_leading         : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);
  signal pos_fc_trailing        : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);

  signal valid_leading          : std_logic_vector(kNumInput -1 downto 0);
  signal valid_trailing         : std_logic_vector(kNumInput -1 downto 0);
  signal finecount_leading      : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);
  signal finecount_trailing     : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);

  -- Ofs correction --
  constant kBitHbLsb        : integer:= 13;
  signal reduced_ofs        : signed(kWidthFineCount downto 0);

  signal valid_cleading         : std_logic_vector(kNumInput -1 downto 0);
  signal finecount_cleading     : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);
  signal valid_ctrailing        : std_logic_vector(kNumInput -1 downto 0);
  signal finecount_ctrailing    : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);

  -- TDC channel mask --
  signal valid_data_mask_l        : std_logic_vector(kNumInput -1 downto 0);
  signal valid_data_mask_t        : std_logic_vector(kNumInput -1 downto 0);

  -- TDC delay buffer --
  signal valid_data_delay_l     : std_logic_vector(kNumInput -1 downto 0);
  signal valid_data_delay_t     : std_logic_vector(kNumInput -1 downto 0);
  signal finecount_data_delay_l   : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);
  signal finecount_data_delay_t   : FineCountArrayType(kNumInput-1 downto 0)(kWidthFineCount-1 downto 0);


  -- Trigger emulation --
  signal valid_data_trigger_l   : std_logic_vector(kNumInput -1 downto 0);
  signal valid_data_trigger_t   : std_logic_vector(kNumInput -1 downto 0);

  -- Delimiter inserter --
  signal dtiming_hb_l           : TimingArrayType(kNumInput-1 downto 0)(kWidthTiming-1 downto 0);
  signal dtiming_hb_t           : TimingArrayType(kNumInput-1 downto 0)(kWidthTiming-1 downto 0);

  signal valid_inserter_l       : std_logic_vector(kNumInput -1 downto 0);
  signal valid_inserter_t       : std_logic_vector(kNumInput -1 downto 0);
  signal dout_inserter_l        : IntDataArrayType(kNumInput-1 downto 0)(kWidthIntData-1 downto 0);
  signal dout_inserter_t        : IntDataArrayType(kNumInput-1 downto 0)(kWidthIntData-1 downto 0);

  -- TOTFilter --
  signal valid_tot_filter_l     : std_logic_vector(kNumInput -1 downto 0);
  signal valid_tot_filter_t     : std_logic_vector(kNumInput -1 downto 0);
  signal dout_tot_filter_l      : DataArrayType(kNumInput-1 downto 0);
  signal dout_tot_filter_t      : DataArrayType(kNumInput-1 downto 0);

type sliceOrgArray is array(kNumInput-1 downto 0) of string(1 to 8);
constant SlicePosition              : sliceOrgArray :=
("X001Y100", "X011Y100", "X019Y100", "X037Y100", "X046Y100", "X057Y100", "X063Y100", "X075Y100",
 --"X084Y000", "X093Y000", "X057Y050", "X063Y050", "X068Y050", "X075Y050", "X084Y050", "X093Y050",
 "X019Y050", "X028Y050", "X037Y050", "X046Y050", "X057Y050", "X067Y050", "X075Y050", "X084Y050",
 --"X019Y050", "X028Y050", "X037Y050", "X046Y050", "X057Y000", "X063Y000", "X068Y000", "X075Y000",
 "X057Y000", "X067Y000", "X075Y000", "X084Y000", "X091Y000", "X100Y000", "X091Y050", "X100Y050",
 "X001Y000", "X011Y000", "X019Y000", "X028Y000", "X037Y000", "X046Y000", "X001Y050", "X011Y050"
 );

  -- Debug --
  attribute mark_debug : boolean;
--  attribute mark_debug of valid_leadingbuffer      : signal is enDEBUG;
--  attribute mark_debug of valid_trailingbuffer     : signal is enDEBUG;

  function GetDebugFlag(index : integer) return boolean is
  begin
    if(index = 1) then
      return true;
    else
      return false;
    end if;
  end function;

begin
  -- =========================== body ===============================
  daq_off_reset <= not daqOn;

  validOut(kNumInput-1 downto 0)            <= valid_tot_filter_l;
  validOut(2*kNumInput-1 downto kNumInput)  <= valid_tot_filter_t;
  dOut(kNumInput-1 downto 0)                <= dout_tot_filter_l;
  dOut(2*kNumInput-1 downto kNumInput)      <= dout_tot_filter_t;

  hitOut    <= valid_raw_leading;

  -- register buffer ------------------------------------------------
  reg_ready_l   <= and_reduce(ready_lut_leading);
  reg_ready_t   <= and_reduce(ready_lut_trailing);

  u_buf : process(baseClk)
  begin
    if(baseClk'event and baseClk = '1') then
      reg_auto_sw   <= regAutoSw;
      reg_through   <= regThrough;
      reg_switch    <= regSwitch;

      regReadyLut   <= reg_ready_l and reg_ready_t;
    end if;
  end process;

  -- Time Stamp -----------------------------------------------------
  -- Timing Unit --
  sig_in_n  <= not sigIn;

  reduced_ofs   <= LaccpFineOffset(kBitHbLsb downto kBitHbLsb-kWidthFineCount);
  gen_tdc : for i in 0 to kNumInput-1 generate
    attribute keep_hierarchy of u_reset_gen_tdc : label is "true";

  begin
    u_tap_inst : entity mylib.HRTimingTaps
      generic map(
        SliceOrigin    => SlicePosition(i)
        )
      port map(
        RST     => rst,
        tdcClk  => tdcClk,
        testModeIn => testModeIn,

        -- input --
        sigIn   => sigIn(i),
        calibIn => calibIn,

        -- output
        TapOut  => tap_out(i)
       );

    --leading_taps(i)   <= tap_out(i);
    --trailing_taps(i)  <= not tap_out(i);
    leading_taps(i)   <= GetInvVector(regEnInv) xor tap_out(i);
    trailing_taps(i)  <= (not GetInvVector(regEnInv)) xor tap_out(i);

--    u_tdcL_Inst : entity mylib.dummyHRTimingDecoder
--      port map(
--        tdcClk      => tdcClk,
--        sysClk      => baseClk,
--
--        -- input --
--        sigIn       => sigIn(i),
--
--        -- output --
--        hdOut       => valid_leading(i),
--        dOut        => finecount_leading(i)
--        );

    u_tdcL_Inst : entity mylib.HRTimingDecoder
      port map(
        tdcRst      => sync_reset_tdc(i),

        tdcClk      => tdcClk,
        sysClk      => baseClk,

        -- input --
        tapIn       => leading_taps(i),


        -- output --
        hdOut       => valid_raw_leading(i),
        dOut        => raw_fc_leading(i)
        );

    u_LUT_L : entity mylib.FineCountLut
      port map(
        RST         => sync_reset,
        CLK         => baseClk,

        -- control bits --
        regThrough  => reg_through,
        regSwitch   => reg_switch,
        regAutoSW   => reg_auto_sw,
        regReady    => ready_lut_leading(i),

        -- module input --
        hdIn        => valid_raw_leading(i),
        dIn         => raw_fc_leading(i),

        -- module output --
        hdOut       => hdout_fcl_leading(i),
        dOut        => dout_fcl_leading(i)
        );

--    u_tdcT_Inst : entity mylib.dummyHRTimingDecoder
--    port map(
--      tdcClk      => tdcClk,
--      sysClk      => baseClk,
--
--      -- input --
--      sigIn       => sig_in_n(i),
--
--      -- output --
--      hdOut       => valid_trailing(i),
--      dOut        => finecount_trailing(i)
--      );

    u_tdcT_Inst : entity mylib.HRTimingDecoder
      port map(
        tdcRst      => sync_reset_tdc(i),

        tdcClk      => tdcClk,
        sysClk      => baseClk,

        -- input --
        tapIn       => trailing_taps(i),

        -- output --
        hdOut       => valid_raw_trailing(i),
        dOut        => raw_fc_trailing(i)
        );

    u_LUT_T : entity mylib.FineCountLut
      port map(
        RST         => sync_reset,
        CLK         => baseClk,

        -- control bits --
        regThrough  => reg_through,
        regSwitch   => reg_switch,
        regAutoSW   => reg_auto_sw,
        regReady    => ready_lut_trailing(i),

        -- module input --
        hdIn        => valid_raw_trailing(i),
        dIn         => raw_fc_trailing(i),

        -- module output --
        hdOut       => hdout_fcl_trailing(i),
        dOut        => dout_fcl_trailing(i)
        );

--    u_semi_buf : process(baseClk)
--    begin
--      if(baseClk'event and baseClk = '1') then
--        semi_count_leading_1(i)  <= raw_fc_leading(i)(kWidthFine+1 downto kWidthFine);
--        semi_count_leading_2(i)  <= semi_count_leading_1(i);
--        semi_count_leading_3(i)  <= semi_count_leading_2(i);
--
--        semi_count_trailing_1(i) <= raw_fc_trailing(i)(kWidthFine+1 downto kWidthFine);
--        semi_count_trailing_2(i) <= semi_count_trailing_1(i);
--        semi_count_trailing_3(i) <= semi_count_trailing_2(i);
--      end if;
--    end process;

    -- Define FineCount of HR-TDC --
    valid_leading(i)      <= hdout_fcl_leading(i);
    valid_trailing(i)     <= hdout_fcl_trailing(i);

    pos_fc_leading(i)     <= dout_fcl_leading(i);
    pos_fc_trailing(i)    <= dout_fcl_trailing(i);
    finecount_leading(i)  <= std_logic_vector(unsigned(fzero) - unsigned(pos_fc_leading(i)));
    finecount_trailing(i) <= std_logic_vector(unsigned(fzero) - unsigned(pos_fc_trailing(i)));
    -- Define FineCount of HR-TDC --

    -- Ofs correction --
    u_ofsc_l : entity mylib.OfsCorrect
      generic map(
        kWidthOfs   => kWidthFineCount+1,
        enDEBUG     => false
      )
      port map(
        clk             => baseClk,
        syncReset       => rst,

        -- LACCP --
        enOfsCorr       => enBypassOfsCorr,
        reducedOfs      => reduced_ofs,

        -- TDC in --
        validIn         => valid_leading(i),
        dInTiming       => finecount_leading(i),

        -- Data out --
        validOut        => valid_cleading(i),
        dOut            => finecount_cleading(i)
      );

    u_ofsc_t : entity mylib.OfsCorrect
      generic map(
        kWidthOfs   => kWidthFineCount+1,
        enDEBUG     => false
      )
      port map(
        clk             => baseClk,
        syncReset       => rst,

        -- LACCP --
        enOfsCorr       => enBypassOfsCorr,
        reducedOfs      => reduced_ofs,

        -- TDC in --
        validIn         => valid_trailing(i),
        dInTiming       => finecount_trailing(i),

        -- Data out --
        validOut        => valid_ctrailing(i),
        dOut            => finecount_ctrailing(i)
      );


    u_reset_gen_tdc   : entity mylib.ResetGen
      port map(rst, tdcClk, sync_reset_tdc(i));

  end generate;

  -- TDC delay buffer ------------------------------------------------
  valid_data_mask_l <= valid_cleading and (not regTdcMaskL);
  valid_data_mask_t <= valid_ctrailing and (not regTdcMaskT);

  u_TDCDelayBufferL : entity mylib.TDCDelayBuffer
    generic map(
      kNumInput => kNumInput,
      enDEBUG   => false
      )
    port map
    (
      -- system --
      clk             => baseClk,
      enBypass        => enBypassDelay,

      -- Data In --
      validIn         => valid_data_mask_l,
      isLeadingIn     => (others => '1'),
      dInTOT          => (others => (others => '0')),
      dIn             => finecount_cleading,

      -- Data Out --
      vaildOut        => valid_data_delay_l,
      isLeadingOut    => open,
      dOutTOT         => open,
      dOut            => finecount_data_delay_l
      );

  u_TDCDelayBufferT : entity mylib.TDCDelayBuffer
    generic map(
      kNumInput => kNumInput,
      enDEBUG   => false
      )
    port map
    (
      -- system --
      clk             => baseClk,
      enBypass        => enBypassDelay,

      -- Data In --
      validIn         => valid_data_mask_t,
      isLeadingIn     => (others => '0'),
      dInTOT          => (others => (others => '0')),
      dIn             => finecount_ctrailing,

      -- Data Out --
      vaildOut        => valid_data_delay_t,
      isLeadingOut    => open,
      dOutTOT         => open,
      dOut            => finecount_data_delay_t
      );

  -- Heartbeat frame definition ------------------------------------------------
  gen_tdcdata : for i in 0 to kNumInput-1 generate
  begin
    valid_data_trigger_l(i)   <= triggerGate and valid_data_delay_l(i);
    valid_data_trigger_t(i)   <= triggerGate and valid_data_delay_t(i);
    dtiming_hb_l(i)           <= hbCount & finecount_data_delay_l(i);
    dtiming_hb_t(i)           <= hbCount & finecount_data_delay_t(i);


    u_delimiterInserterL : entity mylib.DelimiterInserter
      generic map
        (
          enDEBUG         => false
          --enDEBUG       => GetDebugFlag(i)
        )
      port map
      (
        -- system --
        clk             => baseClk,
        syncReset       => sync_reset,
        --userRegIn       => userReg,
        --channelNum      => std_logic_vector(to_unsigned(i+GetChOffset(genChOffset), kWidthChannel)),
        enBypassParing  => enBypassParing,
        signBit         => LaccpFineOffset(LaccpFineOffset'high),

        -- TDC in --
        validIn         => valid_data_trigger_l(i),
        dInTiming       => dtiming_hb_l(i),
        isLeading       => '1',
        dInToT          => (others => '0'),

        -- Delimiter word input --
        validDelimiter  => validDelimiter,
        dInDelimiter    => dInDelimiter,
        daqOn           => daqOn,
        hbfThrottlingOn => hbfThrottlingOn,

        -- output --
        validOut        => valid_inserter_l(i),
        dOut            => dout_inserter_l(i)
      );

    --
    u_delimiterInserterT : entity mylib.DelimiterInserter
      generic map
        (
          enDEBUG         => false
          --enDEBUG       => GetDebugFlag(i)
        )
      port map
      (
        -- system --
        clk             => baseClk,
        syncReset       => sync_reset,
        --userRegIn       => userReg,
        --channelNum      => std_logic_vector(to_unsigned(i+GetChOffset(genChOffset), kWidthChannel)),
        enBypassParing  => enBypassParing,
        signBit         => LaccpFineOffset(LaccpFineOffset'high),

        -- TDC in --
        validIn         => valid_data_trigger_t(i),
        dInTiming       => dtiming_hb_t(i),
        isLeading       => '0',
        dInToT          => (others => '0'),

        -- Delimiter word input --
        validDelimiter  => validDelimiter,
        dInDelimiter    => dInDelimiter,
        daqOn           => daqOn,
        hbfThrottlingOn => hbfThrottlingOn,

        -- output --
        validOut        => valid_inserter_t(i),
        dOut            => dout_inserter_t(i)
      );
  end generate;


  -- Data processing -----------------------------------------------------------
  gen_daqword : for i in 0 to kNumInput-1 generate
  begin

    u_TOTFilterL : entity mylib.TOTFilter
      port map(
        syncReset         => sync_reset or daq_off_reset,
        clk               => baseClk,
        enFilter          => '0',
        minTh             => (others => '0'),
        maxTh             => (others => '0'),
        enZeroThrough     => '0',
        channelNum        => std_logic_vector(to_unsigned(i+GetChOffset(genChOffset), kWidthChannel)),

        -- Data In --
        validIn           => valid_inserter_l(i),
        dIn               => dout_inserter_l(i),

        -- Out --
        validOut          => valid_tot_filter_l(i),
        dOut              => dout_tot_filter_l(i)
      );

    --
    u_TOTFilterT : entity mylib.TOTFilter
      port map(
        syncReset         => sync_reset or daq_off_reset,
        clk               => baseClk,
        enFilter          => '0',
        minTh             => (others => '0'),
        maxTh             => (others => '0'),
        enZeroThrough     => '0',
        channelNum        => std_logic_vector(to_unsigned(i+GetChOffset(genChOffset), kWidthChannel)),

        -- Data In --
        validIn           => valid_inserter_t(i),
        dIn               => dout_inserter_t(i),

        -- Out --
        validOut          => valid_tot_filter_t(i),
        dOut              => dout_tot_filter_t(i)
      );
  end generate;

  -- Reset sequence --
  u_reset_gen_sys   : entity mylib.ResetGen
    port map(rst, baseClk, sync_reset);

end RTL;
