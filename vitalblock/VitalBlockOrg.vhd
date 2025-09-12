library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;

library mylib;
use mylib.defDelimiter.all;
use mylib.defDataBusAbst.all;
use mylib.defHeartBeatUnit.all;

entity VitalBlock is
  generic (
    kTdcType            : string := "LRTDC"; -- "LRTDC" or "HRTDC"
    kNumInput           : integer:= 32;
    kDivisionRatio      : integer:= 4;
    enDEBUG             : boolean := false
  );
  port (
    rst                 : in STD_LOGIC;  -- User reset (synchronous)
    clk                 : in STD_LOGIC;
    daqGateIn           : in std_logic;
    lhbfNumMismatch     : out std_logic; -- Local heartbeat frame num mismatch

    -- ODPBlock input --
    odpWrenIn           : in  std_logic_vector(kNumInput-1 downto 0); -- TDC data write enable
    odpDataIn           : in  DataArrayType(kNumInput-1 downto 0);
    hbCount             : in  std_logic_vector(kWidthHBCount-1 downto 0);

    -- Status output --
    bufferProgFull      : out std_logic;                              -- Incomming buffer prog full flag
    bufferFull          : out std_logic;                              -- Incomming buffer full flag

    -- Throttling status --
    outThrottlingOn     : out std_logic;                              -- Output throttling status
    inThrottlingT2On    : out std_logic;                              -- Input throttling Type2 status

    -- Offset correction --
    rdEnFromOfsCorr     : in std_logic;

    -- Link buf status --
    pfullLinkBufIn      : in std_logic;
    emptyLinkInBufIn    : in std_logic;

    -- output --
    rdenIn              : in  STD_LOGIC;                                  --output fifo read enable
    dataOut             : out STD_LOGIC_VECTOR (kWidthData-1 downto 0);   --output fifo data out
    emptyOut            : out STD_LOGIC;
    almostEmptyOut      : out STD_LOGIC;
    validOut            : out STD_LOGIC                                   --output fifo valid flag
  );
end VitalBlock;

architecture Behavioral of VitalBlock is

  -- System --
  signal sync_reset             : std_logic;

  signal incoming_buf_pfull     : std_logic_vector(kNumInput-1 downto 0);
  signal incoming_buf_full      : std_logic_vector(kNumInput-1 downto 0);
  signal input_throttling_type2_on : std_logic;
  signal output_throttling_on   : std_logic;
  signal local_hbf_num_mismatch : std_logic;

  -- Input Throttling Type2 --
  signal valid_ithrottling      : std_logic_vector(kNumInput-1 downto 0);
  signal dout_ithrottling       : DataArrayType(kNumInput-1 downto 0);
  signal inthrottling_is_working : std_logic_vector(kNumInput-1 downto 0);

  signal t2start_insert_request, t2start_insert_ack   : std_logic_vector(kNumInput-1 downto 0);
  signal t2end_insert_request, t2end_insert_ack   : std_logic_vector(kNumInput-1 downto 0);


  -- incoming unit -> merger unit
  signal rden_incoming          : std_logic_vector(kNumInput-1 downto 0);
  signal dout_incoming          : DataArrayType(kNumInput-1 downto 0);
  signal empty_incoming         : std_logic_vector(kNumInput-1 downto 0);
  signal almost_empty_incoming  : std_logic_vector(kNumInput-1 downto 0);
  signal valid_incoming         : std_logic_vector(kNumInput-1 downto 0);

  -- Merger to OutputThrottling
  signal dout_merger_out        : STD_LOGIC_VECTOR (kWidthData-1 downto 0);   --output fifo data out
  signal valid_merger_out       : STD_LOGIC;
  signal read_enable_to_merger  : STD_LOGIC;

  attribute mark_debug : boolean;
  attribute mark_debug of odpWrenIn         : signal is enDEBUG;

  attribute mark_debug of inthrottling_is_working  : signal is enDEBUG;

  attribute mark_debug of valid_ithrottling  : signal is enDEBUG;
  attribute mark_debug of t2start_insert_request  : signal is enDEBUG;
  attribute mark_debug of t2start_insert_ack  : signal is enDEBUG;
  attribute mark_debug of t2end_insert_request  : signal is enDEBUG;
  attribute mark_debug of t2end_insert_ack  : signal is enDEBUG;

  attribute mark_debug of valid_incoming    : signal is enDEBUG;
  attribute mark_debug of rden_incoming   : signal is enDEBUG;
  attribute mark_debug of empty_incoming  : signal is enDEBUG;

begin

  lhbfNumMismatch <= local_hbf_num_mismatch;

  bufferProgFull  <= '0' when(unsigned(incoming_buf_pfull) = 0) else '1';
  bufferFull      <= '0' when(unsigned(incoming_buf_full) = 0) else '1';
  inThrottlingT2On  <= input_throttling_type2_on;
  outThrottlingOn   <= output_throttling_on;

  input_throttling_type2_on   <= '0' when(unsigned(inthrottling_is_working) = 0) else '1';


  -- Input Throttling Type2 --
  gen_ithrottle_t2 : for i in 0 to kNumInput-1 generate
  begin
    u_ITT2 : entity mylib.InputThrottlingType2
      generic map(
        kChannel  => i,
        enDEBUG   => false
      )
      port map(
        syncReset           => sync_reset or (not daqGateIn),
        clk                 => clk,

        -- status input --
        ibufProgFullIn      => incoming_buf_pfull(i),
        emptyIbufIn         => empty_incoming(i),

        -- Heartbeat count for TDC
        hbCount             => hbCount,

        -- Status output --
        isWorking           => inthrottling_is_working(i),
        t2startReq          => t2start_insert_request(i),
        t2startAck          => t2start_insert_ack(i),
        t2endReq            => t2end_insert_request(i),
        t2endAck            => t2end_insert_ack(i),

        -- Data In --
        validIn             => odpWrenIn(i),
        dIn                 => odpDataIn(i),

        -- Data Out --
        validOut            => valid_ithrottling(i),
        dOut                => dout_ithrottling(i)

      );
  end generate;


  -- IncomingBuffer --
  u_incoming_buffer: entity mylib.IncomingBuffer
    generic map(
      kNumStrInput  => kNumInput,
      enDEBUG       => false
    )
    port map(
      clk             => clk,
      syncReset       => sync_reset,

      odpWrenIn       => valid_ithrottling,
      odpDataIn       => dout_ithrottling,

      bufferProgFull  => incoming_buf_pfull,
      bufferFull      => incoming_buf_full,

      bufRdenIn          => rden_incoming,
      bufDataOut         => dout_incoming,
      bufEmptyOut        => empty_incoming,
      bufAlmostEmptyOut  => almost_empty_incoming,
      bufValidOut        => valid_incoming

    );

  -- MergerBlock --
  gen_hrtdc : if kTdcType = "RAYRAW-TDC" generate
  begin

    output_throttling_on  <= '0';
    validOut              <= valid_merger_out;
    dataOut               <= dout_merger_out;


    u_merger_block: entity mylib.MergerMznBlock
      generic map(
        kNumInput       => kNumInput,
        kDivisionRatio  => kDivisionRatio,
        enDEBUG         => false
      )
      port map(
        clk           => clk,
        syncReset     => sync_reset,

        rdenOut       => rden_incoming,
        dataIn        => dout_incoming,
        emptyIn       => empty_incoming,
        almostEmptyIn => almost_empty_incoming,
        validIn       => valid_incoming,

        rdenIn        => rdenIn,
        dataOut       => dout_merger_out,
        emptyOut      => emptyOut,
        almostEmptyOut => almostEmptyOut,
        validOut      => valid_merger_out
      );
  end generate;

  gen_lrtdc : if kTdcType = "LRTDC" generate
  begin

    read_enable_to_merger   <= '1' when(output_throttling_on = '1') else
                               '0' when(output_throttling_on = '0' and rdEnFromOfsCorr = '0') else
                              rdenIn;

    u_merger_block: entity mylib.MergerBlock
      generic map(
        kNumInput       => kNumInput,
        kDivisionRatio  => kDivisionRatio,
        enDEBUG         => true
      )
      port map(
        clk             => clk,
        syncReset       => sync_reset,
        hbfNumMismatch  => local_hbf_num_mismatch,

        rdenOut         => rden_incoming,
        dataIn          => dout_incoming,
        emptyIn         => empty_incoming,
        almostEmptyIn   => almost_empty_incoming,
        validIn         => valid_incoming,

        rdenIn          => read_enable_to_merger,
        dataOut         => dout_merger_out,
        emptyOut        => emptyOut,
        almostEmptyOut  => almostEmptyOut      ,
        validOut        => valid_merger_out
      );

    u_OutThrottle: entity mylib.OutputThrottling
      generic map(
        enDEBUG => false
      )
      port map(
        syncReset           => sync_reset,
        clk                 => clk,

        -- status input --
        intputThrottlingOn  => input_throttling_type2_on,
        pfullLinkIn         => pfullLinkBufIn,
        emptyLinkIn         => emptyLinkInBufIn,

        -- Status output --
        isWorking           => output_throttling_on,

        -- Data In --
        validIn             => valid_merger_out,
        dIn                 => dout_merger_out,

        -- Data Out --
        validOut            => validOut,
        dOut                => dataOut

      );
  end generate;

  -- Reset sequence --
  u_reset_gen_sys   : entity mylib.ResetGen
    port map(rst, clk, sync_reset);

end Behavioral;
