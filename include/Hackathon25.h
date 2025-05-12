#ifndef HACKATHON25_H
#define HACKATHON25_H

#include <stdint.h>

enum CAN_MSGs {
    Error = 0x020,
    Game = 0x040,
    GameState = 0x050,
    GameFinish = 0x070,
    Die = 0x080,
    Move = 0x090,
    Join = 0x100,
    Leave = 0x101,
    Player = 0x110,
    GameAck = 0x120,
    Rename = 0x500,
    RenameFollow = 0x510
};

struct __attribute__((packed)) MSG_Join {
    uint32_t HardwareID;
};

struct __attribute__((packed)) MSG_Player {
    uint32_t HardwareID;
    uint8_t PlayerID;
};

struct __attribute__((packed)) MSG_Rename {
    uint8_t PlayerID;
    uint8_t size;
    char first6Chars[6];
};

struct __attribute__((packed)) MSG_RenameFollow {
    uint8_t PlayerID;
    char nextChars[7];
};

struct __attribute__((packed)) MSG_Game {
    uint8_t PlayerID_1;
    uint8_t PlayerID_2;
    uint8_t PlayerID_3;
    uint8_t PlayerID_4;
};

struct __attribute__((packed)) MSG_Ack {
    uint8_t PlayerID;
};

struct __attribute__((packed)) MSG_GameState {
    uint8_t Player1_x;
    uint8_t Player1_y;
    uint8_t Player2_x;
    uint8_t Player2_y;
    uint8_t Player3_x;
    uint8_t Player3_y;
    uint8_t Player4_x;
    uint8_t Player4_y;
};

struct __attribute__((packed)) MSG_Move {
    uint8_t PlayerID;
    uint8_t Direction; //`UP` = 1; `RIGHT` = 2; `DOWN` = 3; `LEFT` = 4
};

struct __attribute__((packed)) MSG_Die {
    uint8_t PlayerID;
};

struct __attribute__((packed)) MSG_GameFinish {
    uint8_t PlayerID_1;
    uint8_t PlayerID_1_Pts;
    uint8_t PlayerID_2;
    uint8_t PlayerID_2_Pts;
    uint8_t PlayerID_3;
    uint8_t PlayerID_3_Pts;
    uint8_t PlayerID_4;
    uint8_t PlayerID_4_Pts;
};

struct __attribute__((packed)) MSG_Error {
    uint8_t PlayerID;
    uint8_t ErrorCode;
};


#endif