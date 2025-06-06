#include <vector>
#include "raylib.h"
#include "raymath.h"
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
        float mass=1;
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
    if(balls.empty()) return;
    for(auto& b: balls){
        b->pos={
            .x=b->pos.x+b->velocity.x*dt,
            .y=b->pos.y+b->velocity.y*dt
        };
    }
    //ball to ball collision
    for(int i=0;i<balls.size()-1;i++){
        for(int j=i+1;j<balls.size();j++){
            if(checkCollision(balls[i],balls[j])){
                //to calculate a unit vector along AB
                Vector2 normal={balls[j]->pos.x-balls[i]->pos.x,balls[j]->pos.y-balls[i]->pos.y};
                float len_normal=sqrtf(normal.x*normal.x+normal.y*normal.y);
                normal.x/=len_normal;
                normal.y/=len_normal;

                Vector2 relative_velocity={balls[j]->velocity.x-balls[i]->velocity.x,balls[j]->velocity.y-balls[i]->velocity.y};
                float overlap=balls[i]->radius+balls[j]->radius- 
                                                sqrtf((balls[i]->pos.x - balls[j]->pos.x)*(balls[i]->pos.x - balls[j]->pos.x) 
                                                + (balls[i]->pos.y - balls[j]->pos.y)*(balls[i]->pos.y - balls[j]->pos.y));
                
                float total_mass=balls[i]->mass+balls[j]->mass;
                float coeffA=overlap*balls[j]->mass/total_mass;
                float coeffB=overlap*balls[i]->mass/total_mass;

                //correcting ball positions so they do not sink
                //balls move distance proportional to the other balls mass
                balls[i]->pos.x-=coeffA*normal.x;
                balls[i]->pos.y-=coeffA*normal.y;
                balls[j]->pos.x+=coeffB*normal.x;
                balls[j]->pos.y+=coeffB*normal.y;

                if (Vector2DotProduct(relative_velocity, normal) > 0) continue;
                //correcting velocity using impulse
                float impulse=-(1+1)*(normal.x*relative_velocity.x+normal.y*relative_velocity.y)*
                                    (balls[i]->mass*balls[j]->mass)/total_mass;

                balls[i]->velocity.x-=(impulse/balls[i]->mass)*normal.x;
                balls[i]->velocity.y-=(impulse/balls[i]->mass)*normal.y;
                balls[j]->velocity.x+=(impulse/balls[j]->mass)*normal.x;
                balls[j]->velocity.y+=(impulse/balls[j]->mass)*normal.y;

                balls[i]->color=RED;

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


    InitWindow(SCREEN_WIDTH,SCREEN_HEIGHT,"Physics Engine");
    vector<Ball*> balls;
    for(int i=0;i<1000;i++){

        auto b= new Ball(GetRandomValue(50,900),GetRandomValue(50,900),10,GetRandomValue(-900,900),GetRandomValue(-900,900));
        balls.push_back(b);
    }

 
    SetExitKey(KEY_Q);
   SetTargetFPS(60);

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
