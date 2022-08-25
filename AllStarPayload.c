/*
 * AllStarPayload.c
 *
 *  Created on: Aug 23, 2022
 *      Author: Justin
 */

#include "AllStarPayload.h"

AllstarPayload asp;

void InitASP()
{
    asp.rng_seed = 0xC88F4F11;
    asp.lineup_data[0].stage = 0xC0;
    asp.lineup_data[0].ckind = 0x09; // Marth
    asp.lineup_data[1].stage = 0xC2;
    asp.lineup_data[1].ckind = 0x05; // Bowser
    asp.lineup_data[2].stage = 0xB6;
    asp.lineup_data[2].ckind = 0x04; // Kirby
    asp.lineup_data[3].stage = 0xB5;
    asp.lineup_data[3].ckind = 0x11; // Yoshi
    asp.lineup_data[4].stage = 0xBE;
    asp.lineup_data[4].ckind = 0x0C; // Peach
    asp.lineup_data[5].stage = 0xBE;
    asp.lineup_data[5].ckind = 0x0A; // Mewtwo
    asp.lineup_data[6].stage = 0xC9;
    asp.lineup_data[6].ckind = 0x19; // Ganondorf
    asp.lineup_data[7].stage = 0xC9;
    asp.lineup_data[7].ckind = 0x07; // Luigi
    asp.lineup_data[8].stage = 0xC3;
    asp.lineup_data[8].ckind = 0x16; // Dr. Mario
    asp.lineup_data[9].stage = 0xC3;
    asp.lineup_data[9].ckind = 0x0F; // Jigglypuff
    asp.lineup_data[10].stage = 0xBB;
    asp.lineup_data[10].ckind = 0x0B; // Ness
    asp.lineup_data[11].stage = 0xBB;
    asp.lineup_data[11].ckind = 0x12; // Zelda
    asp.lineup_data[12].stage = 0xC4;
    asp.lineup_data[12].ckind = 0x15; // Young Link
    asp.lineup_data[13].stage = 0xC4;
    asp.lineup_data[13].ckind = 0x17; // Roy
    asp.lineup_data[14].stage = 0xC4;
    asp.lineup_data[14].ckind = 0x14; // Falco
    asp.lineup_data[15].stage = 0xC6;
    asp.lineup_data[15].ckind = 0x18; // Pichu
    asp.lineup_data[16].stage = 0xC6;
    asp.lineup_data[16].ckind = 0x00; // C. Falcon
    asp.lineup_data[17].stage = 0xC6;
    asp.lineup_data[17].ckind = 0x06; // Link
    asp.lineup_data[18].stage = 0xB1;
    asp.lineup_data[18].ckind = 0x08; // Mario
    asp.lineup_data[19].stage = 0xB1;
    asp.lineup_data[19].ckind = 0x02; // Fox
    asp.lineup_data[20].stage = 0xB1;
    asp.lineup_data[20].ckind = 0x0D; // Pikachu
    asp.lineup_data[21].stage = 0xBD;
    asp.lineup_data[21].ckind = 0x0E; // Ice Climbers
    asp.lineup_data[22].stage = 0xBD;
    asp.lineup_data[22].ckind = 0x10; // Samus
    asp.lineup_data[23].stage = 0xBD;
    asp.lineup_data[23].ckind = 0x01; // Donkey Kong
}
