/*
 * Copyright (C) 2010 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "parse.h"
#include "JTAGTAP.h"
#define DEBUG_PRINT(n) Serial.print((n),HEX); Serial.print(" ");
#define BUFFER_SIZE 800




JTAGTAP jtagtap;

static uint16_t m_sdrBitSize;
static uint32_t m_runTest;
static uint8_t m_numRepeats;
static uint8_t m_current_state;
static uint8_t m_endir_state;
static uint8_t m_enddr_state;
static uint8_t  m_tdi[BUFFER_SIZE];
static uint8_t  m_tdo[BUFFER_SIZE];
static uint8_t  m_tdoMask[BUFFER_SIZE];
static uint8_t m_tdo_expected[BUFFER_SIZE];


void state_goto(uint8_t state)
{
  jtagtap.state_goto(state);
  m_current_state = state;
}



bool is_tdo_as_expected() {
	// serial_comm_.DebugBytes("... received  ", tdo_, sdrsize_bytes_);
	for (uint8_t i = 0; i < bitsToBytes(m_sdrBitSize); ++i) {
		uint8_t expected = m_tdo_expected[i] & m_tdoMask[i];
		uint8_t actual = m_tdo[i] & m_tdoMask[i];
		if (expected != actual) {
			// serial_comm_.Debug("... NO MATCH.");
      // Serial.println(".....no match");
      // Serial.print("the num is :");
      // Serial.print(i);
      // Serial.print("-the expected is:");
      // Serial.print(expected,HEX);
      // Serial.print("the actual is:");
      // Serial.println(actual,HEX);
			return false;
		}
    // Serial.print(m_tdo[i],HEX);
    // Serial.print("**");
	}
	// Serial.println("... match!");
	return true;
}




bool sdr(bool should_begin, bool should_end, bool should_check) {

	int attempts_left = m_numRepeats;
	bool matched = false;
	if (should_begin) {
		state_goto(STATE_SHIFT_DR);
	}
  // Serial.print("--the current statatus is:");
  // Serial.print(jtagtap.currentState());
  // delayMicroseconds(3000);
  // delay(100);
  	while (!matched && attempts_left-- >= 0) {
  		jtagtap.shift_td(m_tdi, m_tdo, m_sdrBitSize, should_end);
  		if (should_check) {
  			if (is_tdo_as_expected()) {
  				matched = true;
          // Serial.println("the device is matched..");
  			} else {
  				state_goto(STATE_PAUSE_DR);
  				state_goto(STATE_SHIFT_DR);
  				state_goto(STATE_RUN_TEST_IDLE);
  				jtagtap.wait_time(m_runTest);
  				state_goto(STATE_SHIFT_DR);
          // Serial.println("the device is not matched, try again..");
  			}
  		}
  	}


	if (should_check && !matched) {
		// Serial.println("SDR check failed.");
		return false;
	}
  if (should_end && matched) {
		if (!m_runTest) {
			state_goto(m_enddr_state);
		} else {
			state_goto(STATE_RUN_TEST_IDLE);
			jtagtap.wait_time(m_runTest);
		}
	}
  //led BLINK
  if(digitalRead(6))
    digitalWrite(6, LOW);
  else
    digitalWrite(6, HIGH);
	return true;
}

/**XCOMPLETE**/
ParseStatus gotXCOMPLETE() {
	// Serial.println("XCOMPLETE");
  Serial.println(analogRead(24));

	return PARSE_SUCCESS;
}

//--------------------------------------
/**XSIR**/
ParseStatus gotXSIR(uint8_t sirNumBits, const uint8_t *sirBitmap) {
 // get the value of the tdi
  memcpy(m_tdi,sirBitmap,bitsToBytes(sirNumBits));
	state_goto(STATE_SHIFT_IR);

	jtagtap.shift_td(m_tdi, m_tdo, sirNumBits, true);
  if(!m_runTest)
  {
    state_goto(m_endir_state);
  }
  else{
    state_goto(STATE_RUN_TEST_IDLE);
		jtagtap.wait_time(m_runTest);
  }

	// print out the instruction
  #ifdef DEBUG_SERIAL_PRINT
	Serial.print("XSIR ");
	DEBUG_PRINT(sirNumBits);
	for ( uint8_t i = 0; i < bitsToBytes(sirNumBits); i++ ) {
		DEBUG_PRINT(m_tdi[i]);
	}
	Serial.println();
  #endif
	return PARSE_SUCCESS;
}

//------------------------------------
/**XTDOMASK**/
ParseStatus gotXTDOMASK(uint16_t maskNumBits, const uint8_t *maskBitmap) {

  //get the value of the tdomask
	memcpy(m_tdoMask,maskBitmap,bitsToBytes(maskNumBits));
	//print out the instruction
  #ifdef DEBUG_SERIAL_PRINT
  Serial.print("XTDOMASK ");
	for ( uint8_t i = 0; i < bitsToBytes(maskNumBits); i++ ) {
		DEBUG_PRINT(m_tdoMask[i]);
	}
  Serial.println();
  #endif
	return PARSE_SUCCESS;
}

//----------------------------------------
/**XRUNTEST**/
ParseStatus gotXRUNTEST(uint32_t runTest)
{
	//get the value of the runTest
	m_runTest = runTest;
	//print out the instruction
  #ifdef DEBUG_SERIAL_PRINT
	Serial.print("XRUNTEST ");
	DEBUG_PRINT((runTest >> 24) & 0x000000FF);
	DEBUG_PRINT((runTest >> 16) & 0x000000FF);
	DEBUG_PRINT((runTest >> 8) & 0x000000FF);
	DEBUG_PRINT(runTest & 0x000000FF);
  Serial.println();
  #endif
	return PARSE_SUCCESS;
}

//-------------------------------------
/**XREPEAT**/
ParseStatus gotXREPEAT(uint8_t numRepeats) {
	//get the value of the runTest
  m_numRepeats = numRepeats;
	//print out the instruction
	// Serial.print("XREPEAT ");
	// DEBUG_PRINT(numRepeats);
  // Serial.println();
	return PARSE_SUCCESS;
}

/**XSDRSIZE**/
ParseStatus gotXSDRSIZE(uint16_t sdrSize) {
	//get the value of the sdrBitSize
	m_sdrBitSize = sdrSize;
	//print out the instruction
	// Serial.print("XSDRSIZE ");
	// DEBUG_PRINT(0x00);
	// DEBUG_PRINT(0x00);
	// DEBUG_PRINT(sdrSize >> 8);
	// DEBUG_PRINT(sdrSize & 0x00FF);
  // Serial.println();
	return PARSE_SUCCESS;
}

//----------------------------------------------------
/**XSDRTDO**/
ParseStatus gotXSDRTDO(uint16_t tdoNumBits, const uint8_t *tdoBitmap, const uint8_t *tdoMask)
{
  bool result=false;
	//get the value of the m_tdo
  memcpy(m_tdi,tdoBitmap, bitsToBytes(tdoNumBits));
  memcpy(m_tdo_expected,(tdoBitmap+bitsToBytes(tdoNumBits)), bitsToBytes(tdoNumBits));
  result = sdr(true, true, true);
	//print out the instruction
  // Serial.print("the prestate is:");
  // Serial.print(jtagtap.currentState());
  #ifdef DEBUG_SERIAL_PRINT
	Serial.print("XSDRTDO ");
	for ( uint8_t i = 0; i < bitsToBytes(tdoNumBits); i++ ) {
		DEBUG_PRINT(m_tdi[i]);
	}
  for ( uint8_t i = 0; i < bitsToBytes(tdoNumBits); i++ ) {
		DEBUG_PRINT(m_tdo_expected[i]);
	}
  Serial.println();
  #endif
	if(result)
  {
    return PARSE_SUCCESS;
  }
  else{

    return PARSE_CALLBACK_XSDRTDO;
  }

}


ParseStatus gotXSDRB(uint16_t tdoNumBits, const uint8_t *tdoBitmap) {
	// Serial.print("XSDRB ");
	// for ( uint8_t i = 0; i < bitsToBytes(tdoNumBits); i++ ) {
	// 	DEBUG_PRINT(*tdoBitmap++);
	// }
  // Serial.println();
	return PARSE_SUCCESS;
}
ParseStatus gotXSDRC(uint16_t tdoNumBits, const uint8_t *tdoBitmap) {
	// Serial.print("XSDRC ");
	// for ( uint8_t i = 0; i < bitsToBytes(tdoNumBits); i++ ) {
	// 	DEBUG_PRINT(*tdoBitmap++);
	// }
  // Serial.println();
	return PARSE_SUCCESS;
}
ParseStatus gotXSDRE(uint16_t tdoNumBits, const uint8_t *tdoBitmap) {
	// Serial.print("XSDRE ");
	// for ( uint8_t i = 0; i < bitsToBytes(tdoNumBits); i++ ) {
	// 	DEBUG_PRINT(*tdoBitmap++);
	// }
  // Serial.println();
	return PARSE_SUCCESS;
}

//------------------------------
/**XSTATE**/
ParseStatus gotXSTATE(TAPState tapState) {
 //print out the instruction
	// Serial.print("XSTATE ");
	// DEBUG_PRINT(tapState);
  // Serial.println();
	//handle the state need to goto
	state_goto(tapState);
	return PARSE_SUCCESS;
}

//--------------------------
/**XENDIR**/
ParseStatus gotXENDIR(uint8_t endIR) {
  //get the value of the xendir state
  if(endIR)
    m_endir_state = STATE_PAUSE_IR;
  else
    m_endir_state = STATE_RUN_TEST_IDLE;
  //print out the instruction
	// Serial.print("XENDIR ");
	// Serial.print(m_endir_state);
  // Serial.println();
	return PARSE_SUCCESS;
}

//-----------------------------
/**XENDDR**/
ParseStatus gotXENDDR(uint8_t endDR) {
  //get the value of the xendir state
  if(endDR)
    m_enddr_state = STATE_PAUSE_DR;
  else
    m_enddr_state = STATE_RUN_TEST_IDLE;
  //print out the instruction
	// Serial.print("XENDDR ");
	// Serial.print(m_enddr_state);
  // Serial.println();
	return PARSE_SUCCESS;
}

//---------------------
/**XWAIT**/
ParseStatus gotXWAIT(uint8_t numByteWaits,const uint8_t *waitBitmap){
  uint8_t m_waitBitMap[10];
  uint32_t m_waitTimes=0;;
  memcpy(m_waitBitMap,waitBitmap,numByteWaits);
  state_goto(m_waitBitMap[0]);
  uint8_t end_state = m_waitBitMap[1];
  m_waitTimes &= 0x00000000;
  m_waitTimes |= ((uint32_t)(m_waitBitMap[2])&(0x000000FF))<<24;
  m_waitTimes |=((uint32_t)(m_waitBitMap[3])&(0x000000FF))<<16;
  m_waitTimes |= ((uint32_t)(m_waitBitMap[4])&(0x000000FF))<<8;
  m_waitTimes |= ((uint32_t)(m_waitBitMap[5])&(0x000000FF));
  jtagtap.wait_time(m_waitTimes);
  state_goto(end_state);
  // print out the instruction
  // Serial.print("XWAIT ");
  // for ( uint8_t i = 0; i < numByteWaits; i++ ) {
	// 	DEBUG_PRINT(*waitBitmap++);
	// }
  // Serial.println();
  return PARSE_SUCCESS;
}


#ifdef PARSE_HAVE_CALLBACKS
const ParseCallbacks m_callbacks = {
	gotXTDOMASK,
	gotXREPEAT,
	gotXRUNTEST,
	gotXSIR,
	gotXSDRSIZE,
	gotXSDRTDO,
	gotXSDRB,
	gotXSDRC,
	gotXSDRE,
	gotXCOMPLETE,
	gotXSTATE,
	gotXENDIR,
	gotXENDDR
};
#endif
