#include <vector>
#include "raylib.h"
using namespace std;
#define WALL_WIDTH 5
#define WALL_HEIGHT 5
#define PLAY 0
#define PAUSE 1
#define EDIT 2



class Ball
{

    public:
        Color color=WHITE;
        Vector2 pos;
        Vector2 velocity;
        Vector2 acceleration;
        float radius;
        int mass;
        Ball(int x,int y,float radius,float velocityX,float velocityY){
            this->pos=(Vector2){(float)x,(float)y};
            this->velocity=(Vector2){(float)velocityX,(float)velocityY};
            this->radius=radius;
        }

        
};
bool checkCollision(Ball* b1,Ball* b2){
    if(
        pow(b1->radius+b2->radius,2)>=( pow((b1->pos.x-b2->pos.x),2) + pow((b1->pos.y-b2->pos.y),2))
    ) return true;

    return false;
}

void UpdateBalls(vector<Ball*> balls,float dt){
    for(auto& b: balls){
        b->pos={
            .x=b->pos.x+b->velocity.x*dt,
            .y=b->pos.y+b->velocity.y*dt
        };
    }
    //ball to ball collision
    for(int i=0;i<balls.size()-1;i++){
        for(int j=0;j<balls.size();j++){
            if(checkCollision(balls[i],balls[j])){
                

            }
        }
    }
    //ball to wall collision
    for(auto& b: balls){
        bool collisionX=false;
        bool collisionY=false;
        float prevx=b->pos.x-b->velocity.x*dt;
        float prevy=b->pos.y-b->velocity.y*dt;
        if(b->pos.x>=GetScreenWidth()-WALL_WIDTH-b->radius){
            collisionX=true;
            float t=(1/(prevx-b->pos.x))*(GetScreenWidth()-WALL_WIDTH-b->radius-b->pos.x);
            b->pos.y=t*prevy+(1-t)*b->pos.y;
            b->pos.x=GetScreenWidth()-WALL_WIDTH-b->radius;
        }else if(b->pos.x<=WALL_WIDTH+b->radius){
            collisionX=true;
            float t=(1/(prevx-b->pos.x))*(WALL_WIDTH+b->radius-b->pos.x);
            b->pos.y=t*prevy+(1-t)*b->pos.y;
            b->pos.x=WALL_WIDTH+b->radius;
        }

        if(b->pos.y>=GetScreenHeight()-WALL_HEIGHT-b->radius){
            collisionY=true;
            float t=(1/(prevy-b->pos.y))*(GetScreenHeight()-WALL_HEIGHT-b->radius-b->pos.y);
            b->pos.x=t*prevx+(1-t)*b->pos.x;
            b->pos.y=GetScreenHeight()-WALL_HEIGHT-b->radius;
        }else if(b->pos.y<=WALL_HEIGHT+b->radius){
            collisionY=true;
            float t=(1/(prevy-b->pos.y))*(WALL_HEIGHT+b->radius-b->radius-b->pos.y);
            b->pos.x=t*prevx+(1-t)*b->pos.x;
            b->pos.y=WALL_HEIGHT+b->radius;

        
        }


        if(collisionX) b->velocity.x*=-1;
        if(collisionY) b->velocity.y*=-1;
    }


}
void RenderBalls(vector<Ball*> balls){
    for(auto& b: balls){
        DrawCircle(b->pos.x,b->pos.y,b->radius,b->color);
    }

}


int main(){
    int STATE=PLAY;
    int SCREEN_WIDTH=GetScreenWidth();
    int SCREEN_HEIGHT=GetMonitorHeight(0);

    Ball* b1=new Ball(100,100,10,50,0);

    InitWindow(SCREEN_WIDTH,SCREEN_HEIGHT,"Physics Engine");
    vector<Ball*> balls;
    balls.push_back(b1);
    Ball* b2=new Ball(100,200,10,1100,0);
    balls.push_back(b2);
    Ball* b3=new Ball(100,250,10,100,1300); Ball* b4=new Ball(100,225,10,50,15);
    balls.push_back(b3);balls.push_back(b4);

 
    SetExitKey(KEY_Q);

    while(!WindowShouldClose()){
        float dt=GetFrameTime();

        BeginDrawing();

        ClearBackground(WHITE);
        DrawRectangle(WALL_WIDTH,WALL_HEIGHT,GetMonitorWidth(0)-2*WALL_WIDTH,GetMonitorHeight(0)-2*WALL_HEIGHT,BLACK);
        DrawFPS(0,0);

        UpdateBalls(balls,dt);

        RenderBalls(balls);
  


        EndDrawing();



    }

    return 0;
}
