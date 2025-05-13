#include <Arduino.h>
#include <CAN.h>
#include "Hackathon25.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#define GRID_SIZE 64
#define PLAYERS 4
#define MAX_TRAIL 2500

int player_sequence_no = 0;
int game_counter =0;

typedef struct { int x, y; } Position;

const int dx[4] = {-1, 0, 1, 0}; // LEFT, UP, RIGHT, DOWN
const int dy[4] = {0, 1, 0, -1};
//Initial conditions
uint64_t grid[GRID_SIZE] = {0}; //Bitpacked grid to trace trails, Complete grid is free
Position tails[PLAYERS];        //To track players new position
Position trails[PLAYERS][MAX_TRAIL]; //Trails of all the players
int trail_lengths[PLAYERS];
int is_alive[PLAYERS] = {1, 1, 1, 1};
int last_dirs[PLAYERS] = {0, 0, 0, 0};

const uint32_t hardware_ID = (*(RoReg *)0x008061FCUL);
uint8_t player_ID = 0;
uint8_t game_ID = 0;
char name[21] = "Level Shifters";


// Function prototypes
void send_Join();
void rcv_Player();
void rename_function();
void send_GameAck();
void send_Move();
void die();

void set_grid(int x, int y, int v) {
// Grid is set to 1 if a player moves to (x,y)
// Grid is set to 0 if a player is dead
    x = (x + GRID_SIZE) % GRID_SIZE;
    y = (y + GRID_SIZE) % GRID_SIZE;
    if (v) grid[y] |= (1ULL << x);
    else grid[y] &= ~(1ULL << x);
}

int get_grid(int x, int y) {
// return the value of the grid in (x,y) coordinate
    x = (x + GRID_SIZE) % GRID_SIZE;
    y = (y + GRID_SIZE) % GRID_SIZE;
    return (grid[y] >> x) & 1;
}

void clear_trail(int pid) {
//Once a player is dead, the complete trail of that player is cleared
    for (int i = 0; i < trail_lengths[pid]; i++) {
        int x = trails[pid][i].x, y = trails[pid][i].y;
        int still_used = 0;
        for (int p = 0; p < PLAYERS && !still_used; p++) {
            if (p == pid || !is_alive[p]) continue;
            for (int j = 0; j < trail_lengths[p]; j++) {
                if (trails[p][j].x == x && trails[p][j].y == y) {
                    still_used = 1;
                    break;
                }
            }
        }
        if (!still_used) set_grid(x, y, 0);
    }
    trail_lengths[pid] = 0;
}
// Predicts the next head positions of all opponent players based on their last known direction
// Parameters:
// - predicted_heads: Array to store predicted head positions for each player
// - player_seq_num: ID of the current player (excluded from prediction)
void predict_opponents_heads(Position predicted_heads[PLAYERS], int player_seq_num) {
    for (int p = 0; p < PLAYERS; p++) {
        if (!is_alive[p] || p == player_seq_num) continue;
        int dir = last_dirs[p];
        predicted_heads[p].x = (tails[p].x + dx[dir] + GRID_SIZE) % GRID_SIZE;
        predicted_heads[p].y = (tails[p].y + dy[dir] + GRID_SIZE) % GRID_SIZE;
    }
}

// Checks if a given position is predicted to be an opponent's head
// Parameters:
// - x, y: Coordinates to check
// - predicted_heads: Array of predicted opponent head positions
// - player_seq_num: ID of the current player (excluded from check)
// Returns: 1 if position is a predicted head, 0 otherwise
int is_predicted_head(int x, int y, Position predicted_heads[PLAYERS], int player_seq_num) {
    for (int p = 0; p < PLAYERS; p++) {
        if (!is_alive[p] || p == player_seq_num) continue;
        if (predicted_heads[p].x == x && predicted_heads[p].y == y) return 1;
    }
    return 0;
}

// Performs BFS to calculate the reachable area from a starting position
// Parameters:
// - sx, sy: Starting coordinates
// - depth_limit: Maximum BFS depth (e.g., 6)
// - predicted_heads: Array of predicted opponent head positions
// - player_seq_num: ID of the current player
// - wall_dist: Pointer to store minimum distance to grid edges
// Returns: Number of reachable cells (area)
int bfs_area(int sx, int sy, int depth_limit, Position predicted_heads[PLAYERS], int player_seq_num, int *wall_dist) {
    uint8_t visited[GRID_SIZE][GRID_SIZE] = {{0}};
    int queue_x[GRID_SIZE * GRID_SIZE], queue_y[GRID_SIZE * GRID_SIZE], queue_d[GRID_SIZE * GRID_SIZE];
    int front = 0, back = 0, area = 0, min_wall = GRID_SIZE * 2;

    sx = (sx + GRID_SIZE) % GRID_SIZE;
    sy = (sy + GRID_SIZE) % GRID_SIZE;

    if (get_grid(sx, sy) || is_predicted_head(sx, sy, predicted_heads, player_seq_num)) return 0;

    queue_x[back] = sx;
    queue_y[back] = sy;
    queue_d[back] = 0;
    back++;
    visited[sy][sx] = 1;
    area = 1;

    int wx = sx < GRID_SIZE / 2 ? sx : GRID_SIZE - 1 - sx;
    int wy = sy < GRID_SIZE / 2 ? sy : GRID_SIZE - 1 - sy;
    min_wall = wx + wy;

    while (front < back) {
        int x = queue_x[front], y = queue_y[front], d = queue_d[front];
        front++;
        if (d >= depth_limit) continue;
        // Explore four directions (LEFT, UP, RIGHT, DOWN)
        for (int dir = 0; dir < 4; dir++) {
            int nx = (x + dx[dir] + GRID_SIZE) % GRID_SIZE;
            int ny = (y + dy[dir] + GRID_SIZE) % GRID_SIZE;
            // Skip if cell is visited, occupied, or predicted to be an opponent's head
            if (visited[ny][nx] || get_grid(nx, ny) || is_predicted_head(nx, ny, predicted_heads, player_seq_num)) continue;
            // Mark cell as visited and enqueue it
            visited[ny][nx] = 1;
            queue_x[back] = nx;
            queue_y[back] = ny;
            queue_d[back] = d + 1;
            back++;
            area++;

            int wx2 = nx < GRID_SIZE / 2 ? nx : GRID_SIZE - 1 - nx;
            int wy2 = ny < GRID_SIZE / 2 ? ny : GRID_SIZE - 1 - ny;
            int wall = wx2 + wy2;
            if (wall < min_wall) min_wall = wall;

            if (area > 80) {
                *wall_dist = min_wall;
                return area; // Early exit
            }
        }
    }
    *wall_dist = min_wall;
    return area;
}

// Decides the next move for the current player
// Parameters:
// - player_seq_num: ID of the current player
// Returns: Move direction (1=UP, 2=RIGHT, 3=DOWN, 4=LEFT)
int decide_move(int player_seq_num) {
    int x = tails[player_seq_num].x, y = tails[player_seq_num].y;
    Position predicted_heads[PLAYERS] = {{0}};
    predict_opponents_heads(predicted_heads, player_seq_num);

    int best_dir = -1, best_score = -1, best_wall = GRID_SIZE * 2;
    const int output_map[4] = {4, 1, 2, 3}; // Maps internal dir (0=LEFT, 1=UP, 2=RIGHT, 3=DOWN) to output (4, 1, 2, 3)

    for (int d = 0; d < 4; d++) { // Evaluate in order: LEFT, UP, RIGHT, DOWN
        int nx = (x + dx[d] + GRID_SIZE) % GRID_SIZE;
        int ny = (y + dy[d] + GRID_SIZE) % GRID_SIZE;

        if (get_grid(nx, ny) || is_predicted_head(nx, ny, predicted_heads, player_seq_num)) continue; // Unsafe

        int wall_dist;
        int score = bfs_area(nx, ny, 6, predicted_heads, player_seq_num, &wall_dist);
        // Update best move if this move has larger area or same area but closer to edge
        if (score > best_score || (score == best_score && wall_dist < best_wall)) {
            best_score = score;
            best_wall = wall_dist;
            best_dir = d;
        }
    }
    // If no safe move, return the last direction used
    if (best_dir == -1) return last_dirs[player_sequence_no];
    return output_map[best_dir]; // Map internal direction to new output
}

void update_grid(const int *new_positions, int player_seq_num) {
    int np = 0;
    for (int p = 0; p < PLAYERS; p++) {
        if (p == player_seq_num) continue;
        int nx = new_positions[np++], ny = new_positions[np++];
        if (nx == 255 && ny == 255) {
            if (is_alive[p]) {
                is_alive[p] = 0;
                clear_trail(p);
            }
            continue;
        }
        if (!is_alive[p]) continue;
        int old_x = tails[p].x, old_y = tails[p].y;
        for (int d = 0; d < 4; d++) {
            if (((old_x + dx[d] + GRID_SIZE) % GRID_SIZE) == nx &&
                ((old_y + dy[d] + GRID_SIZE) % GRID_SIZE) == ny) {
                last_dirs[p] = d;
                break;
            }
        }
        if (get_grid(nx, ny)) {
            is_alive[p] = 0;
            clear_trail(p);
            continue;
        }
        tails[p].x = nx;
        tails[p].y = ny;
        if (trail_lengths[p] < MAX_TRAIL)
            trails[p][trail_lengths[p]++] = (Position){nx, ny};
        set_grid(nx, ny, 1);
    }
    if (!is_alive[player_seq_num]) {
        printf("Our player is dead. Exiting.\n");
        //exit(0);
    }
    int move_output = decide_move(player_seq_num);
    // Map output (1=UP, 2=RIGHT, 3=DOWN, 4=LEFT) to internal (0=LEFT, 1=UP, 2=RIGHT, 3=DOWN)
    const int input_map[5] = {-1, 1, 2, 3, 0}; // Index 0 unused, 1->1, 2->2, 3->3, 4->0
    int move = input_map[move_output];
    int x = tails[player_seq_num].x, y = tails[player_seq_num].y;
    int nx = (x + dx[move] + GRID_SIZE) % GRID_SIZE;
    int ny = (y + dy[move] + GRID_SIZE) % GRID_SIZE;
    last_dirs[player_seq_num] = move;
    if (get_grid(nx, ny)) {
        is_alive[player_seq_num] = 0;
        clear_trail(player_seq_num);
        printf("Our player is dead. Exiting.\n");
    }
    tails[player_seq_num].x = nx;
    tails[player_seq_num].y = ny;
    if (trail_lengths[player_seq_num] < MAX_TRAIL)
        trails[player_seq_num][trail_lengths[player_seq_num]++] = (Position){nx, ny};
    set_grid(nx, ny, 1);
}


// Setting the 1st 6 elements of the Array as the Opponents coordinates based on our player_seq_num
void filterArrayInPlace(int arr[], int player_seq_num) {
  int remove_idx1 = player_seq_num * 2;
  int remove_idx2 = player_seq_num * 2 + 1;
  if (remove_idx1 >= 8 || remove_idx2 >= 8) {
      printf("Indices to remove are out of bounds.\n");
      return;
  }
  for (int i = remove_idx2 + 1; i < 8; i++) {
      arr[i - 2] = arr[i]; 
  }
}

// CAN receive callback
void onReceive(int packetSize) {
  if (packetSize) {
    switch(CAN.packetId()) {      
      case Player:
        Serial.println("CAN: Received Player packet");
        rcv_Player();
        break;
      case Game:
        Serial.println("CAN: Game received");
        send_GameAck();
        break;
      case GameState:
        Serial.println("CAN: Game State received");
        send_Move();
        break;
      case Die:
        Serial.println("CAN: Die received");
        die();
        break;
      default:
        Serial.println("CAN: Received unknown packet");
        break;
    }
  }
}

// CAN setup
bool setupCan(long baudRate) {
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, false);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, true);

    if (!CAN.begin(baudRate)) {
        return false;
    }
    return true;
}

// Setup
void setup() {
    Serial.begin(115200);
    
    Serial.println("Initializing CAN bus...");
    if (!setupCan(500000)) {
        Serial.println("Error: CAN initialization failed!");
        while (1);
    }
    Serial.println("CAN bus initialized successfully."); 

    CAN.onReceive(onReceive);

    delay(1000);
    send_Join();
    
}

// Loop remains empty, logic is event-driven via CAN callback
void loop() {}

// Send JOIN packet via CAN
void send_Join(){
    MSG_Join msg_join;
    msg_join.HardwareID = hardware_ID;

    CAN.beginPacket(Join);
    CAN.write((uint8_t*)&msg_join, sizeof(MSG_Join));
    CAN.endPacket();
}

// Receive player information
void rcv_Player(){
    MSG_Player msg_player;
    CAN.readBytes((uint8_t*)&msg_player, sizeof(MSG_Player));

    if(msg_player.HardwareID == hardware_ID){
        player_ID = msg_player.PlayerID;
        Serial.printf("Player ID recieved\n");
        rename_function();
    }

    Serial.printf("Received Player packet | Player ID received: %u | Own Player ID: %u | Hardware ID received: %u | Own Hardware ID: %u\n", 
        msg_player.PlayerID, player_ID, msg_player.HardwareID, hardware_ID);
}

// Send RENAME packet via CAN
void rename_function(){

    MSG_Rename msg_rename;
    msg_rename.PlayerID = player_ID;
    msg_rename.size = strlen(name);
    memcpy(msg_rename.first6Chars, name, 6);
    
    MSG_RenameFollow msg_followname;
    msg_followname.PlayerID = player_ID;

    CAN.beginPacket(Rename);
    CAN.write((uint8_t*)&msg_rename, sizeof(MSG_Rename));
    CAN.endPacket();

    // Send RenameFollow for remaining characters
    for (int offset = 6; offset < msg_rename.size; offset += 7) {
      MSG_RenameFollow followMsg;
      followMsg.PlayerID = player_ID;

      for (int i = 0; i < 7; i++) {
          followMsg.nextChars[i] = (offset + i < msg_rename.size) ? name[offset + i] : ' ';
      }
      memcpy(msg_followname.nextChars, followMsg.nextChars, 7);

      CAN.beginPacket(RenameFollow);
      CAN.write((uint8_t*)&msg_followname, sizeof(MSG_RenameFollow));
      CAN.endPacket();
    }  
}

//Send ACK once GAME is received
void send_GameAck(){
  
  MSG_Game msg_game;
  CAN.readBytes((uint8_t*)&msg_game, sizeof(MSG_Game));
  int arr[4]={msg_game.PlayerID_1, msg_game.PlayerID_2, msg_game.PlayerID_3, msg_game.PlayerID_4};
  for(int i=0; i<GRID_SIZE;i++){
    grid[i] = 0;
    
  }
  for (int i =0; i<4; i++){
    //Once GameAck is received reset all the States to initial conditions
    clear_trail(i);
    last_dirs[i] = 0;
    is_alive[i] = 1;
    tails[i].x = 0;
    tails[i].y = 0;
    trail_lengths[i] = 0;
    if(arr[i]==player_ID){
      player_sequence_no = i;
      
    }
  }

  MSG_Ack msg_gameack;
  msg_gameack.PlayerID =player_ID;
  CAN.beginPacket(GameAck);
  CAN.write((uint8_t*)&msg_gameack, sizeof(MSG_Ack));
  CAN.endPacket();
}

// send Move Command when GameState is received
void send_Move(){
  
  MSG_GameState msg_gamestate;
  CAN.readBytes((uint8_t*)&msg_gamestate, sizeof(MSG_GameState));
  
  int arr[8]={msg_gamestate.Player1_x, 
              msg_gamestate.Player1_y, 
              msg_gamestate.Player2_x, 
              msg_gamestate.Player2_y,
              msg_gamestate.Player3_x, 
              msg_gamestate.Player3_y, 
              msg_gamestate.Player4_x, 
              msg_gamestate.Player4_y,
              };
  
    tails[player_sequence_no].x = arr[player_sequence_no*2];
    tails[player_sequence_no].y = arr[player_sequence_no*2+1];
    if (trail_lengths[player_sequence_no] < MAX_TRAIL)
        trails[player_sequence_no][trail_lengths[player_sequence_no]++] = (Position){arr[player_sequence_no*2], arr[player_sequence_no*2+1]};
    set_grid(arr[player_sequence_no*2], arr[player_sequence_no*2 +1], 1);

    // Get Player positions from GameState information
    filterArrayInPlace(arr, player_sequence_no);

    MSG_Move msg_move;
    msg_move.PlayerID = player_ID;

    //For the initial movement select a random direction
    int direction_array[4] = {(rand() % 2) ? 1 : 4,2,2,(rand() % 2) ? 1 : 4};
    if(game_counter == 0){
    msg_move.Direction = direction_array[player_sequence_no];
    game_counter = game_counter+1;
    }

    else{
    //For the next movement use the algorithm, as only after the 1st move, Opponents coordinates are known
    msg_move.Direction = decide_move(player_sequence_no); 
    }

    CAN.beginPacket(Move);
    CAN.write((uint8_t*)&msg_move, sizeof(MSG_Move));
    CAN.endPacket();
    //After the new coordinates is sent, Update the grid to track the trails
    update_grid(arr, player_sequence_no);

}

// When player dies, Die message is received
void die(){

  MSG_Die msg_die;
  CAN.readBytes((uint8_t*)&msg_die, sizeof(MSG_Die));
  if(msg_die.PlayerID == player_ID){
    Serial.println("Died\n");
  }

}