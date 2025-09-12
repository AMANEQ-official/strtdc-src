library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

library mylib;
--use mylib.defChannel.all;
use mylib.defDelimiter.all;
use mylib.defDataBusAbst.all;

entity IncomingBuffer is
  generic (
    kNumStrInput    : integer := 32;
    enDEBUG         : boolean := false
  );
  port (
    clk               : in  STD_LOGIC;
    syncReset         : in  STD_LOGIC;  -- synchronous reset

    -- input  (ODP block)
    odpWrenIn         : in  STD_LOGIC_VECTOR (kNumStrInput-1 downto 0); -- Write enable from ODP
    odpDataIn         : in  DataArrayType(kNumStrInput-1 downto 0);

    -- flag   (system)
    bufferProgFull    : out STD_LOGIC_VECTOR (kNumStrInput-1 downto 0); --incomming FIFO prog full
    bufferFull        : out STD_LOGIC_VECTOR (kNumStrInput-1 downto 0); --incomming FIFO full

    -- output (merger unit)
    bufRdenIn         : in  STD_LOGIC_VECTOR (kNumStrInput-1 downto 0); --fifo read enable
    bufDataOut        : out DataArrayType(kNumStrInput-1 downto 0);
    bufEmptyOut       : out STD_LOGIC_VECTOR (kNumStrInput-1 downto 0); --fifo empty flag
    bufAlmostEmptyOut : out STD_LOGIC_VECTOR (kNumStrInput-1 downto 0); --fifo almost empty flag
    bufValidOut       : out STD_LOGIC_VECTOR (kNumStrInput-1 downto 0)  --fifo valid flag
  );
end IncomingBuffer;

architecture Behavioral of IncomingBuffer is

  -- Flag --
  signal is_delimiter       : std_logic_vector(kNumStrInput-1 downto 0);  -- indicate that the TDC data is lost in this delimiter frame

  -- Incoming FIFO --
  signal wren_fifo         : std_logic_vector(kNumStrInput-1 downto 0);
  signal din_fifo          : DataArrayType(kNumStrInput-1 downto 0);
  signal full_fifo         : std_logic_vector(kNumStrInput-1 downto 0);
  --signal almost_full_fifo  : std_logic_vector(kNumStrInput-1 downto 0);
  signal wr_ack_fifo       : std_logic_vector(kNumStrInput-1 downto 0);

  constant kNumBitDepthIncomingFifo : integer  := 9;
  type dDateCountIncomingFifoType is array ( integer range kNumStrInput-1 downto 0) of std_logic_vector(kNumBitDepthIncomingFifo-1 downto 0); -- for count the data count of incoming FIFO
  --signal data_count_fifo   : dDateCountIncomingFifoType;
  signal prog_full_fifo    : std_logic_vector(kNumStrInput-1 downto 0);

  component incomingFifo is
    PORT(
      clk         : in  STD_LOGIC;
      srst        : in  STD_LOGIC;

      wr_en       : in  STD_LOGIC;
      din         : in  STD_LOGIC_VECTOR (kWidthData-1 DOWNTO 0);
      full        : out STD_LOGIC;
      --almost_full : out STD_LOGIC;

      rd_en       : in  STD_LOGIC;
      dout        : out STD_LOGIC_VECTOR (kWidthData-1 DOWNTO 0);
      empty       : out STD_LOGIC;
      almost_empty: out STD_LOGIC;
      valid       : out STD_LOGIC;

      --data_count  : out STD_LOGIC_VECTOR (kNumBitDepthIncomingFifo-1 DOWNTO 0);
      prog_full   : out STD_LOGIC
    );
    end component;

  attribute mark_debug : boolean;
  --attribute mark_debug of flag_1st_delimiter   : signal is enDEBUG;
  --attribute mark_debug of flag_data_lost       : signal is enDEBUG;
  --attribute mark_debug of wren_fifo            : signal is enDEBUG;
  --attribute mark_debug of is_delimiter         : signal is enDEBUG;
  --attribute mark_debug of din_fifo             : signal is enDEBUG;
  --attribute mark_debug of data_count_fifo      : signal is enDEBUG;
  attribute mark_debug of prog_full_fifo       : signal is enDEBUG;
  --attribute mark_debug of full_fifo      : signal is enDEBUG;
  --attribute mark_debug of almost_full_fifo      : signal is enDEBUG;

begin

  --bufferProgFull  <= '0' when unsigned(prog_full_fifo) = 0 else '1';
  bufferProgFull  <= prog_full_fifo;
  bufferFull      <= full_fifo;

  for_process :for i in kNumStrInput-1 downto 0 generate
  begin

    -- outputfifo

    --din_fifo(i)  <= odpDataIn(i);
    outputfifo_process : process(clk)
    begin
      if(clk'event and clk = '1') then
        if(syncReset = '1') then
          wren_fifo(i) <= '0';
        else
          if(odpWrenIn(i) = '1') then -- There are data from the ODP block
            din_fifo(i)  <= odpDataIn(i);
            if(checkUsrData(odpDataIn(i)(kPosHbdDataType'range)) = false)then -- delimiter word
              is_delimiter(i) <= '1';
              wren_fifo(i)    <= '1';
            elsif(prog_full_fifo(i) /= '0') then  -- incoming FIFO is almost full
              wren_fifo(i)    <= '0';
            else                          -- TDC data
              wren_fifo(i)    <= '1';
            end if;
          else
            is_delimiter(i)   <= '0';                        -- no data
            wren_fifo(i)      <= '0';
          end if;
        end if;
      end if;
    end process;

    -- incoming FIFO
    u_incomingFifo: incomingFifo port map(
      clk         => clk,
      srst        => syncReset,

      wr_en       => wren_fifo(i),
      din         => din_fifo(i),
      full        => full_fifo(i),
      --almost_full => almost_full_fifo(i),

      rd_en       => bufRdenIn(i),
      dout        => bufDataOut(i),
      empty       => bufEmptyOut(i),
      almost_empty=> bufAlmostEmptyOut (i),
      valid       => bufValidOut(i),

      --data_count  => data_count_fifo(i),
      prog_full   => prog_full_fifo(i)
    );

  end generate for_process;

end Behavioral;
