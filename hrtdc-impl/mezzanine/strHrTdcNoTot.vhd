library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_MISC.ALL;
use ieee.numeric_std.all;

library UNISIM;
use UNISIM.VComponents.all;

library mylib;
use mylib.defBCT.all;
use mylib.defLaccp.all;
use mylib.defHeartBeatUnit.all;
use mylib.defDataBusAbst.all;
use mylib.defTDC.all;
use mylib.defDelimiter.all;
use mylib.defGateGen.all;
use mylib.defThrottling.all;
use mylib.defStrHRTDC.all;

entity StrHrTdcNoTot is
  generic(
    kNumInput       : integer:= 32;
    kDivisionRatio  : integer:= 4;
    kNumScrThr      : integer:= 5;
    enDEBUG         : boolean:= false
    );
  port(
    -- System ----------------------------------------------------
    genChOffset       : in std_logic;

    rst               : in std_logic;
    rstUser           : in std_logic;
    clk               : in std_logic;
    tdcClk            : in std_logic;

    radiationURE      : in std_logic;
    daqOn             : out std_logic;
    scrThrEn          : out std_logic_vector(kNumScrThr-1 downto 0);

    testModeIn        : in std_logic;

    -- Data Link -------------------------------------------------
    linkActive        : in std_logic;

    -- DAQ status ------------------------------------------------
    lHbfNumMismatch   : in std_logic;
    recoveryRstOut    : out std_logic;
--    outThrottlingOn   : in std_logic;

    -- LACCP -----------------------------------------------------
    heartbeatIn       : in std_logic;
    hbCount           : in std_logic_vector(kWidthStrHbc-1 downto 0);
    hbfNumber         : in std_logic_vector(kWidthStrHbf-1 downto 0);
    ghbfNumMismatchIn : in std_logic;
    hbfState          : in HbfStateType;

    LaccpFineOffset   : in signed(kWidthLaccpFineOffset-1 downto 0);

    frameFlagsIn      : in std_logic_vector(kWidthFrameFlag-1 downto 0);

    -- Streaming TDC interface ------------------------------------
    sigIn             : in std_logic_vector(kNumInput-1 downto 0);
    calibIn           : in std_logic;
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
end StrHrTdcNoTot;

architecture RTL of StrHrTdcNoTot is
  attribute mark_debug      : boolean;

  -- System --
  signal sync_reset       : std_logic;
  signal sync_empty_in, sync_out_thrott_on, sync_link_active  : std_logic;

  attribute mark_debug of sync_empty_in      : signal is enDebug;
  attribute mark_debug of sync_out_thrott_on : signal is enDebug;
  attribute mark_debug of sync_link_active   : signal is enDebug;

  signal pre_vital_reset  : std_logic;
  signal request_vital_reset  : std_logic;
  constant kWidthVitalReset   : integer:= 8;
  signal sr_vital_reset       : std_logic_vector(kWidthVitalReset-1 downto 0);
  signal neg_edge_gate, daq_off_reset : std_logic;

  signal laccp_fine_offset    : signed(kWidthLaccpFineOffset-1 downto 0);

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

  signal incoming_buf_pfull         : std_logic;
  signal incoming_buf_full          : std_logic;
  signal local_hbf_num_mismatch     : std_logic;

  --attribute mark_debug of daq_is_running  : signal is enDEBUG;
  attribute mark_debug of trigger_gate    : signal is enDEBUG;
  attribute mark_debug of local_hbf_num_mismatch    : signal is enDEBUG;

  -- Throttling ---------------------------------------------------
  signal hbf_throttling_on          : std_logic;
  signal reg_hbf_throttling_ratio   : std_logic_vector(kNumHbfMode-1 downto 0);
  signal input_throttling_type2_on  : std_logic;
  signal output_throttling_on       : std_logic;

  -- Delimiter ----------------------------------------------------
  signal delimiter_flags        : std_logic_vector(kWidthDelimiterFlag-1 downto 0);
  signal delimiter_data_valid   : std_logic;
  signal delimiter_dout         : std_logic_vector(kWidthIntData-1 downto 0);
  --signal reg_user_for_delimiter : std_logic_vector(kPosHbdUserReg'length-1 downto 0);

  attribute mark_debug of delimiter_data_valid  : signal is enDEBUG;
  attribute mark_debug of delimiter_dout        : signal is enDEBUG;

  -- ODP ----------------------------------------------------------
  signal odp_wren   : std_logic_vector(2*kNumInput-1 downto 0);
  signal odp_dout   : DataArrayType(2*kNumInput-1 downto 0);

  signal reg_tdc_mask     : std_logic_vector(255 downto 0);
  signal reg_enbypass     : std_logic_vector(kWidthBypass-1 downto 0);

  signal reg_through, reg_switch, reg_auto_sw, reg_eninv  : std_logic;

  signal reg_ready_lut, reg_ready_lut_leading, reg_ready_lut_trailing  : std_logic;
  signal ready_lut_leading                : std_logic_vector(kNumInput-1 downto 0);
  signal ready_lut_trailing               : std_logic_vector(kNumInput-1 downto 0);


  -- Vital --------------------------------------------------------
  --signal valid_vital      : std_logic;
  --signal dout_vital       : std_logic_vector(kWidthData-1 downto 0);

  -- bus process --
  signal state_lbus      : BusProcessType;

  -- Debug --
  attribute mark_debug of heartbeatIn     : signal is enDEBUG;
  attribute mark_debug of daq_is_running  : signal is enDEBUG;
  attribute mark_debug of lHbfNumMismatch : signal is enDEBUG;
  attribute mark_debug of self_recovery   : signal is enDEBUG;
  attribute mark_debug of request_vital_reset : signal is enDEBUG;
  attribute mark_debug of pre_vital_reset : signal is enDEBUG;

begin
  -- ======================================================================
  --                                 body
  -- ======================================================================

  u_sync_empty   : entity mylib.synchronizer port map(clk, emptyLinkInBufIn, sync_empty_in);
--  u_sync_outthr  : entity mylib.synchronizer port map(clk, outThrottlingOn,  sync_out_thrott_on);
  u_sync_tcpon   : entity mylib.synchronizer port map(clk, linkActive,       sync_link_active);


  daqOn             <= daq_is_running;
  recoveryRstOut    <= pre_vital_reset;
  scrThrEn          <= scr_thr_on;
  throttling_on(0)  <= input_throttling_type2_on or output_throttling_on or hbf_throttling_on;
  throttling_on(1)  <= '0';
  throttling_on(2)  <= input_throttling_type2_on;
  throttling_on(3)  <= output_throttling_on;
  throttling_on(4)  <= hbf_throttling_on;

  local_hbf_num_mismatch  <= lhbfNumMismatch;

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
        elsif(sync_empty_in = '1' and wait_count = 0) then
          self_recovery <= '0';
        end if;

        if(heartbeatIn = '1') then
          if(self_recovery = '0' and hbfState = kActiveFrame and sync_link_active = '1') then
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

  delimiter_flags(kIndexFrameFlag1)       <= frameFlagsIn(0);
  delimiter_flags(kIndexFrameFlag2)       <= frameFlagsIn(1);

  -- Delimiter generation --block --
  --laccp_fine_offset <= LaccpFineOffset when(reg_enbypass(kIndexOfsCorr) = '1') else (others => '0');
  u_DelimiterGen: entity mylib.DelimiterGenerator
    generic map(
      enDEBUG     => false
    )
    port map(
      syncReset         => sync_reset,
      clk               => clk,

      -- Status input ----------------------------------
      flagsIn           => delimiter_flags,

      -- LACCP -----------------------------------------
      hbCount           => hbCount,
      hbfNumber         => hbfNumber,
      --LaccpFineOffset   => laccp_fine_offset,
      signBit           => LaccpFineOffset(LaccpFineOffset'high),

      -- Delimiter data output --
      validDelimiter    => delimiter_data_valid,
      dOutDelimiter     => delimiter_dout
    );

  -- ODP block --
  u_ODPBlock: entity mylib.ODPBlockNoTot
    generic map(
      kNumInput     => kNumInput,
      enDEBUG       => false
    )
    port map(
      -- System --
      genChOffset   => genChOffset,

      rst           => rst,
      tdcClk        => tdcClk,
      baseClk       => clk,
      hitOut        => hitOut,
      --userReg       => reg_user_for_delimiter,

      -- LACCP --
      LaccpFineOffset   => LaccpFineOffset,

      -- Control registers --
      regThrough      => reg_through,
      regAutoSW       => reg_auto_sw,
      regSwitch       => reg_switch,
      regReadyLut     => reg_ready_lut,
      regEnInv        => reg_eninv,

      regTdcMask     => reg_tdc_mask(kNumInput-1 downto 0),

      enBypassDelay   => reg_enbypass(kIndexDelay),
      enBypassParing  => '0',
      enBypassOfsCorr => reg_enbypass(kIndexOfsCorr),

      enTotFilter     => '0',
      enTotZeroThrough => '0',
      totMinTh        => (others => '0'),
      totMaxTh        => (others => '0'),

      testModeIn      => testModeIn,

      -- Data flow control --
      daqOn           => daq_is_running,
      hbfThrottlingOn => hbf_throttling_on,
      triggerGate     => trigger_gate,

      -- Heartbeat counter for TDC --
      hbCount         => hbCount,

      -- Delimiter word I/F --
      validDelimiter  => delimiter_data_valid,
      dInDelimiter    => delimiter_dout,

      -- Signal input --
      sigIn           => sigIn,
      calibIn         => calibIn,

      -- DAQ data output --
      validOut        => odp_wren,
      dOut            => odp_dout
    );

  -- vital block --
  u_VitalBlock: entity mylib.VitalBlock
    generic map(
      kTdcType        => "LRTDC",
      kNumInput       => 2*kNumInput,
      kDivisionRatio  => kDivisionRatio,
      enDEBUG         => false
    )
    port map(
      clk                 => clk,
      rst                 => rstUser or pre_vital_reset or daq_off_reset,
      daqGateIn           => daq_is_running,
      lhbfNumMismatch     => open,

      -- ODPBlock input --
      odpWrenIn           => odp_wren,
      odpDataIn           => odp_dout,
      hbCount             => hbCount,

      -- Hbd flag --
      bufferProgFull      => incoming_buf_pfull,
      bufferFull          => incoming_buf_full,

      -- Throttling status --
      outThrottlingOn     => output_throttling_on,
      inThrottlingT2On    => input_throttling_type2_on,

      -- Offset correction --
      rdEnFromOfsCorr     => '1',

      -- Link buf status --
      pfullLinkBufIn      => pfullLinkBufIn,
      emptyLinkInBufIn    => emptyLinkInBufIn,

      -- Output --
      rdenIn              => dataRdEn,
      dataOut             => dataOut,
      emptyOut            => open,
      almostEmptyOut      => open,
      validOut            => dataRdValid
    );


  -- bus process -------------------------------------------------------------
  u_BusProcess : process(clk)
  begin
    if(clk'event and clk = '1') then
      if(sync_reset = '1') then
        reg_through       <= '0';
        reg_auto_sw       <= '0';
        reg_switch        <= '0';
        reg_eninv         <= '0';
        reg_tdc_mask      <= (others => '0');
        reg_enbypass      <= (others => '0');

        reg_emumode_on    <= (others => '0');
        reg_trigger_delay <= (others => '0');
        reg_trigger_width <= (others => '0');

        reg_hbf_throttling_ratio  <= (others => '0');

        --reg_user_for_delimiter    <= (others => '0');

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
            if(addrLocalBus(kNonMultiByte'range) = kControll(kNonMultiByte'range)) then
              reg_through     <= dataLocalBusIn(kIndexThrough);
              reg_auto_sw     <= dataLocalBusIn(kIndexAutoSW);
              reg_eninv       <= dataLocalBusIn(kIndexEnInv);
              state_lbus      <= Done;

            elsif(addrLocalBus(kNonMultiByte'range) = kReqSwitch(kNonMultiByte'range)) then
              state_lbus      <= Execute;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMask(kNonMultiByte'range)) then
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
              state_lbus      <= Done;

            elsif(addrLocalBus(kNonMultiByte'range) = kEnBypass(kNonMultiByte'range)) then
              reg_enbypass    <= dataLocalBusIn;
              state_lbus      <= Done;

            elsif(addrLocalBus(kNonMultiByte'range) = kTriggerEmuControl(kNonMultiByte'range)) then
              reg_emumode_on  <= dataLocalBusIn(kTriggerMode'range);
              state_lbus      <= Done;

            elsif(addrLocalBus(kNonMultiByte'range) = kTrgGateDelay(kNonMultiByte'range)) then
              reg_trigger_delay <= dataLocalBusIn;
              state_lbus      <= Done;

            elsif(addrLocalBus(kNonMultiByte'range) = kTrgGateWidth(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  reg_trigger_width(7 downto 0)   <= dataLocalBusIn;
                when k2ndByte =>
                  reg_trigger_width(15 downto 8)   <= dataLocalBusIn;
                when others =>
                  null;
              end case;
              state_lbus      <= Done;

            elsif(addrLocalBus(kNonMultiByte'range) = kHbfThrottControl(kNonMultiByte'range)) then
              reg_hbf_throttling_ratio <= dataLocalBusIn(kNumHbfMode-1 downto 0);
              state_lbus      <= Done;

--            elsif(addrLocalBus(kNonMultiByte'range) = kHbdUserReg(kNonMultiByte'range)) then
--              case addrLocalBus(kMultiByte'range) is
--                when k1stByte =>
--                  reg_user_for_delimiter(7 downto 0)    <= dataLocalBusIn;
--                when k2ndByte =>
--                  reg_user_for_delimiter(15 downto 8)   <= dataLocalBusIn;
--                when others =>
--                  null;
--              end case;
--              state_lbus      <= Done;

            elsif(addrLocalBus(kNonMultiByte'range) = kSelfRecoveryMode(kNonMultiByte'range)) then
              reg_self_recovery_mode <= dataLocalBusIn(0);
              state_lbus      <= Done;

            else
              state_lbus      <= Done;
            end if;

          when Read =>
            if(addrLocalBus(kNonMultiByte'range) = kControll(kNonMultiByte'range)) then
              dataLocalBusOut <= "00000" & reg_eninv & reg_auto_sw & reg_through;

            elsif(addrLocalBus(kNonMultiByte'range) = kStatus(kNonMultiByte'range)) then
              dataLocalBusOut <= "0000000" & reg_ready_lut;

            elsif(addrLocalBus(kNonMultiByte'range) = kTdcMask(kNonMultiByte'range)) then
              case addrLocalBus(kMultiByte'range) is
                when k1stByte =>
                  dataLocalBusOut <= reg_tdc_mask(7 downto 0);
                when k2ndByte =>
                  dataLocalBusOut <= reg_tdc_mask(15 downto 8);
                when k3rdByte =>
                  dataLocalBusOut <= reg_tdc_mask(23 downto 16);
                when k4thByte =>
                  dataLocalBusOut <= reg_tdc_mask(31 downto 24);
                when others =>
                  null;
              end case;

            elsif(addrLocalBus(kNonMultiByte'range) = kEnBypass(kNonMultiByte'range)) then
              dataLocalBusOut <= reg_enbypass;

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
              dataLocalBusOut <= X"ee";
            end if;

            state_lbus    <= Done;

          when Execute =>
            reg_switch    <= '1';
            state_lbus    <= Finalize;

          when Finalize =>
            reg_switch    <= '0';
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
    port map(rstUser, clk, sync_reset);

end RTL;
