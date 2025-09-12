library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

package defDataBusAbst is

  -- Definition of data word  -----------------------------------------------------------------
  constant kWidthData         : integer  := 64;    -- width of the data
  type DataArrayType is array(natural range <> ) of std_logic_vector(kWidthData-1 downto 0);

  -- TDC Data Array ---------------------------------------------------------------------------
  type FineCountArrayType is array (natural range <>) of std_logic_vector;
  type ChannelArrayType   is array (natural range <>) of std_logic_vector;
  type TimingArrayType    is array (natural range <>) of std_logic_vector;
  type TOTArrayType       is array (natural range <>) of std_logic_vector;

  -- ADC Data Array ---------------------------------------------------------------------------
  type AdcArrayType       is array (natural range <>) of std_logic_vector;

  -- Definition of the internal data word ----------------------------------------------------
  type IntDataArrayType is array(natural range <>) of std_logic_vector;

end package;
