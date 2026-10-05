#include <vector>
#include <algorithm>
#include <cmath>
#include "raylib.h"
#include "raymath.h"
using namespace std;
#define WALL_WIDTH 5
#define WALL_HEIGHT 5
#define PLAY 0
#define PAUSE 1
#define EDIT 2
#define COEFF_OF_RESTITUTION 1.0f
class Ball
{

    public:
        Color color=WHITE;
        Vector2 pos;
        Vector2 velocity;
        Vector2 acceleration={0,0};
        float radius;
        float mass=1;
        Ball(float x,float y,float radius,float velocityX,float velocityY){
            this->pos=(Vector2){x,y};
            this->velocity=(Vector2){velocityX,velocityY};
            this->radius=radius;
        }


};
bool checkCollision(const Ball& b1,const Ball& b2){
    float dx=b1.pos.x-b2.pos.x;
    float dy=b1.pos.y-b2.pos.y;
    float r=b1.radius+b2.radius;
    return dx*dx+dy*dy<r*r;
}

void resolveCollisionElastic(Ball& a,Ball& b){
    //to calculate a unit vector along AB
    Vector2 normal={b.pos.x-a.pos.x,b.pos.y-a.pos.y};
    float len_normal=sqrtf(normal.x*normal.x+normal.y*normal.y);
    if (len_normal == 0) return;
    normal.x/=len_normal;
    normal.y/=len_normal;

    Vector2 relative_velocity={b.velocity.x-a.velocity.x,b.velocity.y-a.velocity.y};
    float overlap=a.radius+b.radius-len_normal;

    float total_mass=a.mass+b.mass;
    float coeffA=overlap*b.mass/total_mass;
    float coeffB=overlap*a.mass/total_mass;

    //correcting ball positions so they do not sink
    //balls move distance proportional to the other balls mass
    a.pos.x-=coeffA*normal.x;
    a.pos.y-=coeffA*normal.y;
    b.pos.x+=coeffB*normal.x;
    b.pos.y+=coeffB*normal.y;

    if (Vector2DotProduct(relative_velocity, normal) > 0) return;
    //correcting velocity using impulse
    float impulse=-(1+COEFF_OF_RESTITUTION)*(normal.x*relative_velocity.x+normal.y*relative_velocity.y)*
                        (a.mass*b.mass)/total_mass;

    a.velocity.x-=(impulse/a.mass)*normal.x;
    a.velocity.y-=(impulse/a.mass)*normal.y;
    b.velocity.x+=(impulse/b.mass)*normal.x;
    b.velocity.y+=(impulse/b.mass)*normal.y;

    a.color=RED;
    b.color=RED;
}

// Uniform grid stored as flat arrays (counting sort), reused between frames.
// cellStart[c]..cellStart[c+1] indexes into cellBalls for cell c.
struct Grid{
    int cols=0,rows=0;
    float cellSize=1;
    vector<int> cellStart;
    vector<int> cellBalls;
    vector<int> ballCell;

    int cellOf(float x,float y) const{
        int cx=min(cols-1,max(0,(int)floorf(x/cellSize)));
        int cy=min(rows-1,max(0,(int)floorf(y/cellSize)));
        return cy*cols+cx;
    }

    void build(const vector<Ball>& balls,float width,float height){
        // a ball can only touch balls in neighbouring cells if a cell is at least one diameter wide
        float maxRadius=1;
        for(const auto& b: balls) maxRadius=max(maxRadius,b.radius);
        cellSize=2*maxRadius;
        cols=max(1,(int)ceilf(width/cellSize));
        rows=max(1,(int)ceilf(height/cellSize));

        cellStart.assign(cols*rows+1,0);
        ballCell.resize(balls.size());
        cellBalls.resize(balls.size());
        for(size_t i=0;i<balls.size();i++){
            ballCell[i]=cellOf(balls[i].pos.x,balls[i].pos.y);
            cellStart[ballCell[i]+1]++;
        }
        for(int c=0;c<cols*rows;c++) cellStart[c+1]+=cellStart[c];
        vector<int> cursor(cellStart.begin(),cellStart.end()-1);
        for(size_t i=0;i<balls.size();i++) cellBalls[cursor[ballCell[i]]++]=(int)i;
    }
};

void collideCells(vector<Ball>& balls,const Grid& grid,int cellA,int cellB){
    for(int k=grid.cellStart[cellA];k<grid.cellStart[cellA+1];k++){
        // pairs inside the same cell are only visited once (n>k)
        int start=(cellA==cellB)?k+1:grid.cellStart[cellB];
        for(int n=start;n<grid.cellStart[cellB+1];n++){
            Ball& a=balls[grid.cellBalls[k]];
            Ball& b=balls[grid.cellBalls[n]];
            if(checkCollision(a,b)) resolveCollisionElastic(a,b);
        }
    }
}

void UpdateBalls(vector<Ball>& balls,Grid& grid,float dt){
    if(balls.empty()) return;
    float width=(float)GetScreenWidth();
    float height=(float)GetScreenHeight();
    for(auto& b: balls){
        b.pos.x+=b.velocity.x*dt;
        b.pos.y+=b.velocity.y*dt;
    }
    //ball to wall collision: clamp inside the walls and reflect the velocity
    for(auto& b: balls){
        float minX=WALL_WIDTH+b.radius,maxX=width-WALL_WIDTH-b.radius;
        float minY=WALL_HEIGHT+b.radius,maxY=height-WALL_HEIGHT-b.radius;
        if(b.pos.x>maxX){b.pos.x=maxX;b.velocity.x=-fabsf(b.velocity.x);}
        else if(b.pos.x<minX){b.pos.x=minX;b.velocity.x=fabsf(b.velocity.x);}
        if(b.pos.y>maxY){b.pos.y=maxY;b.velocity.y=-fabsf(b.velocity.y);}
        else if(b.pos.y<minY){b.pos.y=minY;b.velocity.y=fabsf(b.velocity.y);}
    }

    //ball to ball collision
    grid.build(balls,width,height);
    // visit each cell together with 4 "forward" neighbours so every one of the
    // 8 neighbours is covered exactly once: right, down-left, down, down-right
    for(int cy=0;cy<grid.rows;cy++){
        for(int cx=0;cx<grid.cols;cx++){
            int c=cy*grid.cols+cx;
            if(grid.cellStart[c]==grid.cellStart[c+1]) continue;
            collideCells(balls,grid,c,c);
            if(cx+1<grid.cols) collideCells(balls,grid,c,c+1);
            if(cy+1<grid.rows){
                if(cx>0) collideCells(balls,grid,c,c+grid.cols-1);
                collideCells(balls,grid,c,c+grid.cols);
                if(cx+1<grid.cols) collideCells(balls,grid,c,c+grid.cols+1);
            }
        }
    }
}
void RenderBalls(const vector<Ball>& balls){
    for(const auto& b: balls){
        DrawCircleV(b.pos,b.radius,b.color);
    }

}


int main(){
    const int SCREEN_WIDTH=1280;
    const int SCREEN_HEIGHT=800;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(SCREEN_WIDTH,SCREEN_HEIGHT,"Physics Engine");
    vector<Ball> balls;
    Grid grid;
    for(int i=0;i<100;i++){
        balls.emplace_back((float)GetRandomValue(50,SCREEN_WIDTH-50),(float)GetRandomValue(50,SCREEN_HEIGHT-50),10.0f,
                           (float)GetRandomValue(-900,900),(float)GetRandomValue(-900,900));
    }


    SetExitKey(KEY_Q);
    SetTargetFPS(60);

    while(!WindowShouldClose()){
        float dt=GetFrameTime();

        UpdateBalls(balls,grid,dt);

        BeginDrawing();
        ClearBackground(WHITE);
        DrawRectangle(WALL_WIDTH,WALL_HEIGHT,GetScreenWidth()-2*WALL_WIDTH,GetScreenHeight()-2*WALL_HEIGHT,BLACK);
        DrawFPS(0,0);
        RenderBalls(balls);
        EndDrawing();



    }

    CloseWindow();
    return 0;
}
