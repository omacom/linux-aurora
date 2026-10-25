/* SPDX-License-Identifier: MIT */
/* Pure board-parser controls: no device opening or EGL calls. */
#define main system_check_main
#include "../mesa-m3-system-check.c"
#undef main
#include <assert.h>

#define CHECK(expected, soc, bytes) \
   assert(board_compat_matches(bytes, sizeof(bytes) - 1, soc) == (expected))

int main(void)
{
   CHECK(true, "t8122", "apple,j613\0apple,t8122\0apple,arm-platform\0");
   CHECK(true, "t6030", "apple,j516s\0apple,t6030\0apple,arm-platform\0");
   CHECK(true, "t6030", "apple,arm-platform\0apple,t6030\0apple,j516s\0");
   CHECK(false, "t8122", "apple,j615\0apple,t8122\0");
   CHECK(false, "t6030", "apple,j514s\0apple,t6030\0");
   CHECK(false, "t6030", "apple,j516s\0apple,t6031\0");
   CHECK(false, "t8122", "apple,j613\0apple,j615\0apple,t8122\0");
   CHECK(false, "t8122", "apple,j613\0apple,j613\0apple,t8122\0");
   CHECK(false, "t6030", "apple,j516s\0apple,t6030\0apple,t6030\0");
   CHECK(false, "t6030", "apple,j516s\0apple,t6030\0apple,t8122\0");
   CHECK(false, "t8122", "apple,j613\0apple,t8122-extra\0");
   CHECK(false, "t8122", "apple,j613\0apple,t8122");
   CHECK(false, "t8122", "apple,j613\0\0apple,t8122\0");
   CHECK(false, "t6030", "apple,t6030\0");
   CHECK(false, "t6030", "apple,j516s\0");
   CHECK(false, "t8140", "apple,j516s\0apple,t6030\0");
   CHECK(false, "t6030", "");
   puts("17 board parser controls passed");
   return 0;
}
