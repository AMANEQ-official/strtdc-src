library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_MISC.ALL;
use ieee.numeric_std.all;

library UNISIM;
use UNISIM.VComponents.all;

library mylib;
--use mylib.defDCR.all;
use mylib.defBCT.all;
use mylib.defLaccp.all;
use mylib.defHeartBeatUnit.all;
use mylib.defDataBusAbst.all;
use mylib.defDelimiter.all;
use mylib.defTDC.all;
use mylib.defGateGen.all;
use mylib.defThrottling.all;
use mylib.defStrLRTDC.all;

entity StrLrTdc is
  generic(
    kTdcType        : string:= "LRTDC";
    kNumInput       : integer:= 32;
    kDivisionRatio  : integer:= 4;
    kNumScrThr      : integer:= 5;
    enDEBUG         : boolean:= false
  );
  port(
    -- System ----------------------------------------------------
    rst               : in std_logic;
    clk               : in std_logic;
    tdcClk            : in std_logic_vector(kNumTdcClock-1 downto 0);

    radiationURE      : in std_logic;
    daqOn             : out std_logic;
    scrThrEn          : out std_logic_vector(kNumScrThr-1 downto 0);

    -- Data Link --------------------------------------------------
    linkActive        : in std_logic;

    -- LACCP ------------------------------------------------------
    heartbeatIn       : in std_logic;
    hbCount           : in std_logic_vector(kWidthStrHbc-1 downto 0);
    hbfNumber         : in std_logic_vector(kWidthStrHbf-1 downto 0);
    ghbfNumMismatchIn : in std_logic;
    hbfState          : in HbfStateType;

    LaccpFineOffset   : in signed(kWidthLaccpFineOffset-1 downto 0);

    frameFlagsIn      : in std_logic_vector(kWidthFrameFlag-1 downto 0);

    -- Streaming TDC interface ------------------------------------
    sigIn             : in std_logic_vector(kNumInput-1 downto 0);
    triggerIn         : in std_logic;
    hitOut            : out std_logic_vector(kNumInput-1 downto 0);

    dataRdEn          : in  std_logic;                                 --output fifo read enable
    dataOut           : out std_logic_vector(kWidthData-1 downto 0);   --output fifo data out
    dataRdValid       : out std_logic;                                 --output fifo valid flag

    -- LinkBuffer interface ---------------------------------------
    pfullLinkBufIn    : in std_logic;
    emptyLinkInBufIn  : in std_logic;

    -- Local bus --
    addrLocalBus        : in LocalAddressType;
    dataLocalBusIn      : in LocalBusInType;
    dataLocalBusOut     : out LocalBusOutType;
    reLocalBus          : in std_logic;
    weLocalBus          : in std_logic;
    readyLocalBus       : out std_logic
  );
end StrLrTdc;

architecture RTL of StrLrTdc is
  attribute mark_debug      : boolean;

  -- System --
  signal sync_reset       : std_logic;

  signal pre_vital_reset  : std_logic;
  signal request_vital_reset  : std_logic;
  constant kWidthVitalReset   : integer:= 8;
  signal sr_vital_reset       : std_logic_vector(kWidthVitalReset-1 downto 0);
  signal neg_edge_gate, daq_off_reset : std_logic;

  -- Internal signal declaration ---------------------------------
  -- Scaler --
  type ThrCountType   is array(kNumScrThr-1 downto 0) of unsigned(kWidthHbCount-1 downto 0);
  constant kMaxThrottCount  : unsigned(kWidthHbCount-1 downto 0):= (others => '1');
  signal thr_count      : ThrCountType;
  signal throttling_on  : std_logic_vector(kNumScrThr-1 downto 0);
  signal scr_thr_on     : std_logic_vector(kNumScrThr-1 downto 0);

  -- DAQ control --
  signal daq_is_running         : std_logic;
  constant kRecoveryWait        : unsigned(3 downto 0):= "0101";
  signal self_recovery          : std_logic;
  signal reg_self_recovery_mode : std_logic;

  signal reg_emumode_on         : std_logic_vector(kTriggerMode'range);
  signal reg_trigger_delay      : std_logic_vector(kWidthTrgDelay-1 downto 0);
  signal reg_trigger_width      : std_logic_vector(kWidthTrgWidth-1 downto 0);
  signal trigger_gate           : std_logic;

  signal incoming_buf_pfull     : std_logic;
  signal incoming_buf_full      : std_logic;
  signal local_hbf_num_mismatch : std_logic;

  attribute mark_debug of daq_is_running  : signal is enDEBUG;
  attribute mark_debug of trigger_gate    : signal is enDEBUG;

  -- Throttling ---------------------------------------------------
  signal hbf_throttling_on          : std_logic;
  signal reg_hbf_throttling_ratio   : std_logic_vector(kNumHbfMode-1 downto 0);
  signal input_throttling_type2_on  : std_logic;
  signal output_throttling_on       : std_logic;

  -- Delimiter ----------------------------------------------------
  signal delimiter_flags        : std_logic_vector(kWidthDelimiterFlag-1 downto 0);
  signal delimiter_data_valid   : std_logic;
  signal delimiter_dout         : std_logic_vector(kWidthData-1 downto 0);
  signal int_delimiter_data     : std_logic_vector(kWidthIntData-1 downto 0);
  signal reg_user_for_delimiter : std_logic_vector(kPosHbdUserReg'length-1 downto 0);

  attribute mark_debug of delimiter_data_valid  : signal is enDEBUG;
  attribute mark_debug of delimiter_dout        : signal is enDEBUG;

  -- ODP ----------------------------------------------------------
  signal odp_wren   : std_logic_vector(kNumInput-1 downto 0);
  signal odp_dout   : DataArrayType(kNumInput-1 downto 0);

  signal reg_tdc_mask     : std_logic_vector(255 downto 0);
  signal reg_enbypass     : std_logic_vector(kWidthBypass-1 downto 0);
  signal reg_tot_filter_control : std_logic_vector(kWidthBypass-1 downto 0);
  signal reg_tot_minth, reg_tot_maxth : std_logic_vector(kWidthTOT-1 downto 0);

  -- Vital --------------------------------------------------------
  signal valid_vital      : std_logic;
  signal dout_vital       : std_logic_vector(kWidthData-1 downto 0);

  -- Replacer -----------------------------------------------------
  signal valid_ofscorr    : std_logic;
  signal dout_ofscorr     : std_logic_vector(kWidthData-1 downto 0);
  signal offset_in_hbd    : signed(LaccpFineOffset'range);

  -- OfsCorr ------------------------------------------------------
  constant kBitHbLsb      : integer:= 13;
  signal reduced_ofs      : signed(kPosTiming'length downto 0);
  signal rden_from_ofscorr  : std_logic;

  function RoundingOff(ofs_in : in signed) return signed is
    variable pulse_1    : signed(ofs_in'length-1 downto 0):= (1 => '1', others => '0');
    variable round_ofs  : signed(ofs_in'length-1 downto 0);
  begin
    if(ofs_in(ofs_in'low) = '1') then
      round_ofs := ofs_in + pulse_1;
      return round_ofs(round_ofs'high downto 1);
    else
      return ofs_in(ofs_in'high downto ofs_in'low+1);
    end if;
  end function;

  -- bus process --
  signal state_lbus      : BusProcessType;

  -- Debug --
  attribute mark_debug of heartbeatIn     : signal is enDEBUG;


begin
  -- ======================================================================
  --                                 body
  -- ======================================================================

  daqOn <= daq_is_running;
  scrThrEn          <= scr_thr_on;
  throttling_on(0)  <= input_throttling_type2_on or output_throttling_on or hbf_throttling_on;
  throttling_on(1)  <= '0';
  throttling_on(2)  <= input_throttling_type2_on;
  throttling_on(3)  <= output_throttling_on;
  throttling_on(4)  <= hbf_throttling_on;

  u_throttling_time : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        thr_count   <= (others => (others => '0'));
      else
        for i in 0 to kNumScrThr-1 loop
          if(throttling_on(i) = '1') then
            thr_count(i)   <= thr_count(i)  +1;

            if(thr_count(i) = kMaxThrottCount) then
              scr_thr_on(i)  <= '1';
            else
              scr_thr_on(i)  <= '0';
            end if;
          else
            scr_thr_on(i)  <= '0';
          end if;
        end loop;
      end if;
    end if;
  end process;


  -- DAQ On/Off --
  u_daq_state : process(clk)
    variable  wait_count  : unsigned(3 downto 0):= (others => '0');
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        daq_is_running      <= '0';
        self_recovery       <= '0';
        request_vital_reset <= '0';
      else
        if(local_hbf_num_mismatch = '1' and reg_self_recovery_mode = '1') then
          wait_count    := kRecoveryWait;
          self_recovery <= '1';
        elsif(emptyLinkInBufIn = '1' and wait_count = 0) then
          self_recovery <= '0';
        end if;

        if(heartbeatIn = '1') then
          if(self_recovery = '0' and hbfState = kActiveFrame and linkActive = '1') then
            daq_is_running  <= '1';
          else
            daq_is_running  <= '0';
          end if;

          if(self_recovery = '1' and wait_count /= 0) then
            wait_count  := wait_count -1;
          end if;

          if(self_recovery = '1' and wait_count = 1) then
            request_vital_reset   <= '1';
          end if;

        else
          request_vital_reset   <= '0';
        end if;
      end if;
    end if;
  end process;

  pre_vital_reset <= sr_vital_reset(kWidthVitalReset-1);
  u_vital_reset : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(request_vital_reset = '1') then
        sr_vital_reset  <= (others => '1');
      else
        sr_vital_reset  <= sr_vital_reset(kWidthVitalReset-2 downto 0) & '0';
      end if;
    end if;
  end process;

  u_daqgate_edge    : entity mylib.EdgeDetector port map(clk, (not daq_is_running), neg_edge_gate);
  u_reset_gen_vital : entity mylib.ResetGen
    generic map(128)
    port map(neg_edge_gate, clk, daq_off_reset);

  -- Trigger emulation mode --
  u_gate : entity mylib.GateGen
    port map(
      syncReset       => sync_reset,
      clk             => clk,

      emuModeOn       => reg_emumode_on,
      delayReg        => reg_trigger_delay,
      widthReg        => reg_trigger_width,

      triggerIn       => triggerIn,
      gateOut         => trigger_gate

    );

  -- Heartbeat frame throttling --
  u_hbf_thro : entity mylib.hbfThrottling
    port map(
      clk                 => clk,

      -- Control registers --
      throttlingRatio     => reg_hbf_throttling_ratio,
      hbfNum              => hbfNumber,

      -- Status output --
      isWorking           => hbf_throttling_on
    );


  -- Delimiter generation --
  delimiter_flags(kIndexRadiationURE)     <= radiationURE;

  delimiter_flags(kIndexOverflow)         <= incoming_buf_full;
  delimiter_flags(kIndexGHbfNumMismatch)  <= ghbfNumMismatchIn;
  delimiter_flags(kIndexLHbfNumMismatch)  <= '0';

  delimiter_flags(kIndexInThrottlingT2)   <= input_throttling_type2_on;
  delimiter_flags(kIndexOutThrottling)    <= '0';
  delimiter_flags(kIndexHbfThrottling)    <= hbf_throttling_on;

  delimiter_flags(kIndexFrameFlag1)       <= frameFlagsIn(1);
  delimiter_flags(kIndexFrameFlag2)       <= frameFlagsIn(0);

  u_DelimiterGen: entity mylib.DelimiterGenerator
    generic map(
      enDEBUG       => false
    )
    port map(
      syncReset         => sync_reset,
      clk               => clk,

      -- Status input ----------------------------------
      flagsIn           => delimiter_flags,

      -- LACCP -----------------------------------------
      hbCount           => hbCount,
      hbfNumber         => hbfNumber,
      signBit           => LaccpFineOffset(LaccpFineOffset'high),
--      LaccpFineOffset   => LaccpFineOffset,

      -- Delimiter data output --
      validDelimiter    => delimiter_data_valid,
      dOutDelimiter     => delimiter_dout
    );

  int_delimiter_data(kPosIHbdDataType'range)  <= delimiter_dout(kPosHbdDataType'range);
  int_delimiter_data(kPosIHbdFlag'range)      <= delimiter_dout(kPosHbdFlag'range);
  int_delimiter_data(kPosIHbdHBFrame'range)   <= delimiter_dout(kPosHbdHBFrame'range);

  -- ODP block --
  u_ODPBlock: entity mylib.ODPBlock
    generic map(
      kNumInput     => kNumInput,
      enDEBUG       => false
    )
    port map(
      -- System --
      rst             => rst,
      tdcClk          => tdcClk,
      baseClk         => clk,
      hitOut          => hitOut,
      --userReg         => reg_user_for_delimiter,
      LaccpFineOffset => LaccpFineOffset,

      -- Control registers --
      tdcMask         => reg_tdc_mask(kNumInput-1 downto 0),
      enBypassDelay   => reg_enbypass(kIndexDelay),
      enBypassParing  => reg_enbypass(kIndexParing),
      --enBypassOfsCorr => reg_enbypass(kIndexOfsCorr),

      enTotFilter     => reg_tot_filter_control(kIndexTotFilter),
      enTotZeroThrough => reg_tot_filter_control(kIndexTotZeroThrough),
      totMinTh        => reg_tot_minth,
      totMaxTh        => reg_tot_maxth,

      -- Data flow control --
      daqOn           => daq_is_running,
      hbfThrottlingOn => hbf_throttling_on,
      triggerGate     => trigger_gate,

      -- Heartbeat counter for TDC --
      hbCount         => hbCount,

      -- Delimiter word I/F --
      validDelimiter  => delimiter_data_valid,
      dInDelimiter    => int_delimiter_data,

      -- Signal input --
      sigIn           => sigIn,

      -- DAQ data output --
      validOut        => odp_wren,
      dOut            => odp_dout
    );

  -- vital block --
  u_VitalBlock: entity mylib.VitalBlock
    generic map(
      kTdcType        => kTdcType,
      kNumInput       => kNumInput,
      kDivisionRatio  => kDivisionRatio,
      enDEBUG         => false
    )
    port map(
      clk                 => clk,
      rst                 => rst or pre_vital_reset or daq_off_reset,
      daqGateIn           => daq_is_running,
      lhbfNumMismatch     => local_hbf_num_mismatch,

      -- ODPBlock input --
      odpWrenIn           => odp_wren,
      odpDataIn           => odp_dout,
      hbCount             => hbCount,

      -- Status output --
      bufferProgFull      => incoming_buf_pfull,
      bufferFull          => incoming_buf_full,

      -- Throttling status --
      outThrottlingOn     => output_throttling_on,
      inThrottlingT2On    => input_throttling_type2_on,

      -- Offset correction --
      rdEnFromOfsCorr     => rden_from_ofscorr,

      -- Link buf status --
      pfullLinkBufIn      => pfullLinkBufIn,
      emptyLinkInBufIn    => emptyLinkInBufIn,

      -- Output --
      rdenIn              => dataRdEn,
      dataOut             => dout_vital,
      emptyOut            => open,
      almostEmptyOut      => open,
      validOut            => valid_vital
    );

  -- OfsCorrection V2 --
  reduced_ofs(kWidthFineCount downto 0) <= LaccpFineOffset(kBitHbLsb-1 downto kBitHbLsb-kWidthFineCount-1) when(reg_enbypass(kIndexOfsCorr) = '0') else (others => '0');
  reduced_ofs(reduced_ofs'high downto kWidthFineCount+1)  <= (others => LaccpFineOffset(LaccpFineOffset'high)) when(reg_enbypass(kIndexOfsCorr) = '0') else (others => '0');
  u_corv2 : entity mylib.OfsCorrectV2
    generic map(
      kWidthOfs           => kPosTiming'length,
      enDEBUG             => false
    )
    port map(
      syncReset           => sync_reset or pre_vital_reset or (not daq_is_running),
      clk                 => clk,
      enBypassOfsCorr     => reg_enbypass(kIndexOfsCorr),
      extendedOfs         => RoundingOff(signed(reduced_ofs)),

      -- Data In --
      rdEnOut             => rden_from_ofscorr,
      validIn             => valid_vital,
      dIn                 => dout_vital,

      -- Data Out --
      validOut            => valid_ofscorr,
      dOut                => dout_ofscorr
    );

  -- Replace 2nd delimiter with new delimiter --
  offset_in_hbd <= LaccpFineOffset when(reg_enbypass(kIndexOfsCorr) = '1') else (others => '0');
  u_replacer : entity mylib.DelimiterReplacer
    port map(
      syncReset           => sync_reset or pre_vital_reset or (not daq_is_running),
      clk                 => clk,
      userReg             => reg_user_for_delimiter,
      LaccpFineOffset     => offset_in_hbd,

      -- Data In --
      validIn             => valid_ofscorr,
      dIn                 => dout_ofscorr,

      -- Data Out --
      validOut            => dataRdValid,
      dOut                => dataOut
    );




  -- bus process -------------------------------------------------------------
  u_BusProcess : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        reg_tdc_mask      <= (others => '0');
        reg_enbypass      <= (others => '0');
        reg_tot_filter_control  <= (others => '0');
        reg_tot_minth           <= (others => '0');
        reg_tot_maxth           <= (others => '0');

        reg_emumode_on    <= (others => '0');
        reg_trigger_delay <= (others => '0');
        reg_trigger_width <= (others => '0');

        reg_hbf_throttling_ratio  <= (others => '0');

        reg_user_for_delimiter    <= (others => '0');

        reg_self_recovery_mode    <= '0';

        state_lbus        <= Init;
      else
        case state_lbus is
          when Init =>
            dataLocalBusOut     <= x"00";
            readyLocalBus       <= '0';
            state_lbus          <= Idle;

          when Idle =>
            readyLocalBus    <= '0';
            if(weLocalBus = '1' or reLocalBus = '1') then
              state_lbus    <= Connect;
            end if;

          when Connect =>
            if(weLocalBus = '1') then
              state_lbus    <= Write;
            else
              state_lbus    <= Read;
            end if;

          when Write =>
            if(addrLocalBus(kNonMultiByte'range) = kTdcMaskMainU(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_tdc_mask(7 downto 0)  <= dataLocalBusIn;
                when k2ndByte =>
                  reg_tdc_mask(15 downto 8)  <= dataLocalBusIn;
                when k3rdByte =>
                  reg_tdc_mask(23 downto 16)  <= dataLocalBusIn;
                when k4thByte =>
                  reg_tdc_mask(31 downto 24)  <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskMainD(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_tdc_mask(39 downto 32)  <= dataLocalBusIn;
                when k2ndByte =>
                  reg_tdc_mask(47 downto 40)  <= dataLocalBusIn;
                when k3rdByte =>
                  reg_tdc_mask(55 downto 48)  <= dataLocalBusIn;
                when k4thByte =>
                  reg_tdc_mask(63 downto 56)  <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskMznU(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_tdc_mask(71 downto 64)  <= dataLocalBusIn;
                when k2ndByte =>
                  reg_tdc_mask(79 downto 72)  <= dataLocalBusIn;
                when k3rdByte =>
                  reg_tdc_mask(87 downto 80)  <= dataLocalBusIn;
                when k4thByte =>
                  reg_tdc_mask(95 downto 88)  <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskMznD(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_tdc_mask(103 downto 96)  <= dataLocalBusIn;
                when k2ndByte =>
                  reg_tdc_mask(111 downto 104)  <= dataLocalBusIn;
                when k3rdByte =>
                  reg_tdc_mask(119 downto 112)  <= dataLocalBusIn;
                when k4thByte =>
                  reg_tdc_mask(127 downto 120)  <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskEx(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_tdc_mask(135 downto 128)  <= dataLocalBusIn;
                when k2ndByte =>
                  reg_tdc_mask(143 downto 136)  <= dataLocalBusIn;
                when k3rdByte =>
                  reg_tdc_mask(151 downto 144)  <= dataLocalBusIn;
                when k4thByte =>
                  reg_tdc_mask(159 downto 152)  <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kEnBypass(kNonMultiByte'range)) then
              reg_enbypass  <= dataLocalBusIn;

            elsif(addrLocalBus(kNonMultiByte'range) = kTotFilterControl(kNonMultiByte'range)) then
              reg_tot_filter_control  <= dataLocalBusIn;

            elsif(addrLocalBus(kNonMultiByte'range) = kTotMinTh(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_tot_minth(7 downto 0)   <= dataLocalBusIn;
                when k2ndByte =>
                  reg_tot_minth(kWidthTOT-1 downto 8)  <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTotMaxTh(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_tot_maxth(7 downto 0)   <= dataLocalBusIn;
                when k2ndByte =>
                  reg_tot_maxth(kWidthTOT-1 downto 8)  <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTriggerEmuControl(kNonMultiByte'range)) then
              reg_emumode_on  <= dataLocalBusIn(kTriggerMode'range);

            elsif(addrLocalBus(kNonMultiByte'range) = kTrgGateDelay(kNonMultiByte'range)) then
              reg_trigger_delay <= dataLocalBusIn;

            elsif(addrLocalBus(kNonMultiByte'range) = kTrgGateWidth(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_trigger_width(7 downto 0)   <= dataLocalBusIn;
                when k2ndByte =>
                  reg_trigger_width(15 downto 8)   <= dataLocalBusIn;
                when k3rdByte =>
                  reg_trigger_width(23 downto 16)   <= dataLocalBusIn;
                when k4thByte =>
                  reg_trigger_width(31 downto 24)   <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kHbfThrottControl(kNonMultiByte'range)) then
              reg_hbf_throttling_ratio <= dataLocalBusIn(kNumHbfMode-1 downto 0);

            elsif(addrLocalBus(kNonMultiByte'range) = kHbdUserReg(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_user_for_delimiter(7 downto 0)    <= dataLocalBusIn;
                when k2ndByte =>
                  reg_user_for_delimiter(15 downto 8)   <= dataLocalBusIn;
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kSelfRecoveryMode(kNonMultiByte'range)) then
              reg_self_recovery_mode <= dataLocalBusIn(0);

            else
              null;
            end if;
            state_lbus      <= Done;

          when Read =>
            if(addrLocalBus(kNonMultiByte'range) = kTdcMaskMainU(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut   <= reg_tdc_mask(7 downto 0);
                when k2ndByte =>
                  dataLocalBusOut   <= reg_tdc_mask(15 downto 8);
                when k3rdByte =>
                  dataLocalBusOut   <= reg_tdc_mask(23 downto 16);
                when k4thByte =>
                  dataLocalBusOut   <= reg_tdc_mask(31 downto 24);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskMainD(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut   <= reg_tdc_mask(39 downto 32);
                when k2ndByte =>
                  dataLocalBusOut   <= reg_tdc_mask(47 downto 40);
                when k3rdByte =>
                  dataLocalBusOut   <= reg_tdc_mask(55 downto 48);
                when k4thByte =>
                  dataLocalBusOut   <= reg_tdc_mask(63 downto 56);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskMznU(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut   <= reg_tdc_mask(71 downto 64);
                when k2ndByte =>
                  dataLocalBusOut   <= reg_tdc_mask(79 downto 72);
                when k3rdByte =>
                  dataLocalBusOut   <= reg_tdc_mask(87 downto 80);
                when k4thByte =>
                  dataLocalBusOut   <= reg_tdc_mask(95 downto 88);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskMznU(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut   <= reg_tdc_mask(103 downto 96);
                when k2ndByte =>
                  dataLocalBusOut   <= reg_tdc_mask(111 downto 104);
                when k3rdByte =>
                  dataLocalBusOut   <= reg_tdc_mask(119 downto 112);
                when k4thByte =>
                  dataLocalBusOut   <= reg_tdc_mask(127 downto 120);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMaskEx(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut   <= reg_tdc_mask(135 downto 128);
                when k2ndByte =>
                  dataLocalBusOut   <= reg_tdc_mask(143 downto 136);
                when k3rdByte =>
                  dataLocalBusOut   <= reg_tdc_mask(151 downto 144);
                when k4thByte =>
                  dataLocalBusOut   <= reg_tdc_mask(159 downto 152);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kEnBypass(kNonMultiByte'range)) then
              dataLocalBusOut   <= reg_enbypass;

            elsif(addrLocalBus(kNonMultiByte'range) = kTotFilterControl(kNonMultiByte'range)) then
              dataLocalBusOut   <= reg_tot_filter_control;

            elsif(addrLocalBus(kNonMultiByte'range) = kTotMinTh(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut   <= reg_tot_minth(7 downto 0);
                when k2ndByte =>
                  dataLocalBusOut   <= reg_tot_minth(15 downto 8);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTotMaxTh(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut   <= reg_tot_maxth(7 downto 0);
                when k2ndByte =>
                  dataLocalBusOut   <= reg_tot_maxth(15 downto 8);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kTriggerEmuControl(kNonMultiByte'range)) then
              dataLocalBusOut <= (kTriggerMode'range => reg_emumode_on, others => '0');

            elsif(addrLocalBus(kNonMultiByte'range) = kTrgGateDelay(kNonMultiByte'range)) then
              dataLocalBusOut <= reg_trigger_delay;

            elsif(addrLocalBus(kNonMultiByte'range) = kTrgGateWidth(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut <= reg_trigger_width(7 downto 0);
                when k2ndByte =>
                  dataLocalBusOut <= reg_trigger_width(15 downto 8);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kHbfThrottControl(kNonMultiByte'range)) then
              dataLocalBusOut <= (kNumHbfMode-1 downto 0 => reg_hbf_throttling_ratio, others => '0');

            elsif(addrLocalBus(kNonMultiByte'range) = kSelfRecoveryMode(kNonMultiByte'range)) then
              dataLocalBusOut <= (0 => reg_self_recovery_mode, others => '0');

            else
              null;
            end if;
            state_lbus    <= Done;

          when Done =>
            readyLocalBus <= '1';
            if(weLocalBus = '0' and reLocalBus = '0') then
              state_lbus  <= Idle;
            end if;

          -- probably this is error --
          when others =>
            state_lbus    <= Init;
        end case;
      end if;
    end if;
  end process u_BusProcess;


  -- Reset sequence --
  u_reset_gen_sys   : entity mylib.ResetGen
    port map(rst, clk, sync_reset);

end RTL;
