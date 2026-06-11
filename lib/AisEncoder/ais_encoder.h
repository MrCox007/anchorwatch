#ifndef AIS_ENCODER_H
#define AIS_ENCODER_H

#include <Arduino.h>

// Minimal AIS (AIVDM) encoder for self-reporting a Class B vessel position,
// mirroring what SignalK's aisreporter sends: type 18 (position report) and
// type 24 (static data, parts A + B). Produces ready-to-send !AIVDM sentences
// for a MarineTraffic / AISHub roaming-station UDP feed.
//
// Verify the output with any AIVDM decoder before trusting it: copy an "AIS>"
// line from the serial monitor into e.g. https://www.aggsoft.com/ais-decoder.htm
class AisEncoder {
public:
  // Class B position report (message 18). Pass sog<0 / cog<0 / heading>=360 for "not available".
  static String positionReport(uint32_t mmsi, double lat, double lon,
                               double sogKnots, double cogDeg, int headingDeg) {
    Bits b;
    b.put(18, 6);                                   // message type 18
    b.put(0, 2);                                    // repeat indicator
    b.put(mmsi, 30);                                // MMSI
    b.put(0, 8);                                    // reserved
    int sog = (sogKnots >= 0 && sogKnots < 102.2) ? (int)(sogKnots * 10 + 0.5) : 1023;
    b.put(sog, 10);                                 // SOG (0.1 kn)
    b.put(0, 1);                                    // position accuracy
    b.putSigned(degToAis(lon), 28);                 // longitude (1/10000 min)
    b.putSigned(degToAis(lat), 27);                 // latitude
    int cog = (cogDeg >= 0 && cogDeg < 360) ? (int)(cogDeg * 10 + 0.5) : 3600;
    b.put(cog, 12);                                 // COG (0.1 deg)
    b.put((headingDeg >= 0 && headingDeg < 360) ? headingDeg : 511, 9); // true heading
    b.put(60, 6);                                   // timestamp (60 = n/a)
    b.put(0, 2);                                    // regional reserved
    b.put(1, 1);                                    // CS unit (Class B "CS")
    b.put(0, 1);                                    // display flag
    b.put(0, 1);                                    // DSC flag
    b.put(1, 1);                                    // band flag
    b.put(1, 1);                                    // message 22 flag
    b.put(0, 1);                                    // assigned
    b.put(0, 1);                                    // RAIM
    b.put(0, 20);                                   // radio status (ignored by aggregators)
    return wrap(b);
  }

  // Static data part A: vessel name (message 24, part 0).
  static String staticA(uint32_t mmsi, const char* name) {
    Bits b;
    b.put(24, 6);
    b.put(0, 2);
    b.put(mmsi, 30);
    b.put(0, 2);                                    // part number 0 (A)
    putText(b, name, 20);                           // vessel name (20 chars)
    return wrap(b);
  }

  // Static data part B: ship type, call sign, dimensions (message 24, part 1).
  static String staticB(uint32_t mmsi, uint8_t shipType, const char* callsign) {
    Bits b;
    b.put(24, 6);
    b.put(0, 2);
    b.put(mmsi, 30);
    b.put(1, 2);                                    // part number 1 (B)
    b.put(shipType, 8);                             // ship type (37 = pleasure craft)
    b.put(0, 18);                                   // vendor ID
    b.put(0, 4);                                    // unit model code
    b.put(0, 20);                                   // serial number
    putText(b, callsign, 7);                        // call sign (7 chars)
    b.put(0, 9);                                    // dimension to bow
    b.put(0, 9);                                    // dimension to stern
    b.put(0, 6);                                    // dimension to port
    b.put(0, 6);                                    // dimension to starboard
    b.put(0, 4);                                    // type of EPFD
    b.put(0, 2);                                    // spare
    return wrap(b);
  }

private:
  // Big-endian bit buffer (max 168 bits = 21 bytes).
  struct Bits {
    uint8_t bytes[24];
    int nbits;
    Bits() { nbits = 0; memset(bytes, 0, sizeof(bytes)); }
    void put(uint32_t value, int len) {
      for (int i = len - 1; i >= 0; i--) {
        if ((value >> i) & 1UL) bytes[nbits / 8] |= (1 << (7 - (nbits % 8)));
        nbits++;
      }
    }
    void putSigned(long value, int len) {
      uint32_t v = (uint32_t)value & (len >= 32 ? 0xFFFFFFFFUL : ((1UL << len) - 1));
      put(v, len);
    }
  };

  // Degrees -> AIS units (1/10000 minute), rounded.
  static long degToAis(double deg) {
    return (long)(deg * 600000.0 + (deg < 0 ? -0.5 : 0.5));
  }

  // ASCII char -> AIS 6-bit value.
  static int sixbitChar(char c) {
    if (c >= 'a' && c <= 'z') c -= 32;             // uppercase
    if (c >= '@' && c <= '_') return c - 64;       // '@'..'_' -> 0..31
    if (c >= ' ' && c <= '?') return c;            // ' '..'?' -> 32..63
    return 0;                                       // unsupported -> '@'
  }

  // Pack a fixed-length text field, padding with '@' (0) after the string ends.
  static void putText(Bits& b, const char* s, int chars) {
    bool end = false;
    for (int i = 0; i < chars; i++) {
      char c = end ? 0 : s[i];
      if (c == 0) { end = true; b.put(0, 6); }
      else b.put(sixbitChar(c), 6);
    }
  }

  // 6-bit armor the bit buffer and wrap it in a checksummed !AIVDM sentence.
  static String wrap(Bits& b) {
    int fill = (6 - (b.nbits % 6)) % 6;
    int groups = (b.nbits + fill) / 6;
    String payload;
    for (int g = 0; g < groups; g++) {
      int val = 0;
      for (int k = 0; k < 6; k++) {
        int bitpos = g * 6 + k;
        int bit = 0;
        if (bitpos < b.nbits) bit = (b.bytes[bitpos / 8] >> (7 - (bitpos % 8))) & 1;
        val = (val << 1) | bit;
      }
      int a = val + 48;
      if (a > 87) a += 8;
      payload += (char)a;
    }
    String body = "AIVDM,1,1,,A,";
    body += payload;
    body += ",";
    body += String(fill);
    uint8_t cs = 0;
    for (size_t i = 0; i < body.length(); i++) cs ^= (uint8_t)body[i];
    char tail[6];
    sprintf(tail, "*%02X", cs);
    return "!" + body + String(tail);
  }
};

#endif // AIS_ENCODER_H
