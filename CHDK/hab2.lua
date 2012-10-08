-- GAGA Camera Control Code
--
-- Copyright (c) 2010 John Graham-Cumming
--
-- Performs the following steps:
--
-- Performs a self-check
-- Waits for a predetermined amount of time
-- Enters loop doing the following:
--   Take a number of photographs in succession
--   Wait a predetermined amount of time

--[[
@title GAGA Camera Control

@param s Start-up delay (secs)
@default s 10

@param c Pictures per iteration 
@default c 1

@param i Iteration delay (secs)
@default i 6

@param f Flight time (hrs)
@default f 4

]]

function stamp()
    return string.format("%4d:%02d:%02d:%02d:%02d:%02d",
    get_time("Y"),
    get_time("M"),
    get_time("D"),
    get_time("h"),
    get_time("m"),
    get_time("s"))
end

ok = 1

function log(m)
  print(m)
  l = io.open("A/gaga.log", "ab")
  if ( l ~= nil ) then
    l:write(string.format("%s:%s\n", stamp(), m))
    l:close()
  end
end

function assert_error(e)
  play_sound(6)
  er = string.format("ER: %s", e)
  log( er )
  ok = 0  
end

function assert_prop(p, v, m)
  pp = get_prop(p)
  log( string.format("Assert property (%i) %i == %i (%s)", p, pp, v, m))
  if ( pp ~= v ) then
    assert_error(m)
  end
end

function assert_eq(a, b, m)
  log( string.format("Assert %i == %i (%s)", a, b, m))
  if ( a ~= b ) then
    assert_error(m)
  end
end

function assert_gt(a, b, m)
  log( string.format("Assert %i > %i (%s)", a, b, m))
  if ( a <= b ) then
    assert_error(m)
  end
end

function assert_lt(a, b, m)
  log( string.format("Assert %i < %i (%s)", a, b, m))
  if ( a >= b ) then
    assert_error(m)
  end
end

-- The sleep function uses microseconds so the s and i need
-- to be converted

ns = (f * 60 * 60 * c) / i
s = s * 1000
i = i * 1000

-- g_free = get_free_disk_space
-- free = g_free/1024

-- g_size = get_disk_size
-- required = (g_size/1024) - 100

log( "GAGA Camera Control" )

-- Now enter a self-check of the manual mode settings

log( "Self-check started" )

assert_prop( 49, -32768, "Not in Auto mode" )
assert_prop(  5,      0, "AF Assist not Off" )
assert_prop(  6,      0, "Focus Mde not Normal" )
-- assert_prop( 21,      0, "Auto Rot not Off" )
assert_prop( 29,      0, "Bracket not None" ) -- Is set to Superfine in CHDK menu, but this isnt reflected in this value.
-- assert_prop( 57,      0, "Picture not Sfine" )
assert_prop( 66,      0, "DateStamp not Off" )
assert_prop( 95,      0, "Digital Zoom not None" )
assert_prop( 102,      0, "Drive Mode not Single" )
assert_prop( 133,      0, "MF not Off" )
assert_prop( 143,      2, "Flash not Off" )
-- assert_prop( 149,    100, "ISO not 100" )
assert_prop( 218,      0, "Picture Sze not L" )
assert_prop( 268,      0, "WB notAuto" )
-- assert_prop( get_time("Y"), 2012, "Unexpected year" ) -- reports wrong year
assert_gt( get_time("h"), 6, "Hour to early" )
assert_lt( get_time("h"), 20, "Hour to late" )
assert_gt( get_vbatt(), 2800, "Batt low" )
-- assert_gt( free, required, "Insuff space" ) -- Cant get this working yet :(

if ( ok == 1 ) then
  sleep(s)
  log( "Starting picture capture" )

  n = 0

  while ( 1 )  do
    tc = c
    while ( tc > 0 ) do
      shoot()
      n = n + 1
      log( string.format("P:%i:Temp:%i:%i:Bt:%i", n, get_temperature(0), get_temperature(1), get_vbatt() ))
      tc = tc - 1
    end
    sleep(i)
  end
end

log( "Done" )
