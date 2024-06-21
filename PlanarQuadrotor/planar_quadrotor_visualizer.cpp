#include "planar_quadrotor_visualizer.h"
#include <cmath>


PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr, float screenWidthMeters, float screenHeightMeters, int screenWidthPixels, int screenHeightPixels)
    : quadrotor_ptr(quadrotor_ptr), screenWidthMeters(screenWidthMeters), screenHeightMeters(screenHeightMeters), screenWidthPixels(screenWidthPixels), screenHeightPixels(screenHeightPixels) {}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x_metr, q_y_metr, q_theta;

    q_x_metr = state[0];
    q_y_metr = state[1];
    q_theta = state[2];

    // Konwersja współrzędnych na piksele
    int q_x = static_cast<int>((q_x_metr / screenWidthMeters) * screenWidthPixels);
    int q_y = static_cast<int>((q_y_metr / screenHeightMeters) * screenHeightPixels);

    //15.5f wysokosc

    float prostokat_dlug =120.0f;
    float prostokat_wys = 10.5f;
    float elipsa_dlug = 40.0f;
    float elipsa_wys = 6.0f;

    
    float rad = q_theta;




    //LG - x,y = -1   //x cały czas na odwrót,więc zmiana obu znaków a y się zmienia z - na + więc zmiana tam przy wysokości zależnej od współrzędnej y
    float x_LG = -prostokat_dlug / 2 * cos(rad) + prostokat_wys / 2 * sin(rad) + q_x;
    float y_LG = prostokat_dlug / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;

    //PG - x=1, y=-1                   //y cia
    float x_PG = prostokat_dlug / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
    float y_PG = -prostokat_dlug / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;




    //te 4 punkty odpowiadają prawemu górnemu rogowi czyli w stanie 0 x=1, y=-1
    float xDDŚ_PHL = prostokat_dlug * 0.8 / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
    float yDDŚ_PHL = -prostokat_dlug * 0.8 / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;
    float xDDŚ_PGHH = prostokat_dlug * 0.8 / 2 * cos(rad) - (75.0f + prostokat_wys) / 2 * sin(rad) + q_x;
    float yDDŚ_PGHH = -prostokat_dlug * 0.8 / 2 * sin(rad) - (75.0f + prostokat_wys) / 2 * cos(rad) + q_y;

    float xDDŚ_LHL = - prostokat_dlug * 0.8 / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
    float yDDŚ_LHL =   prostokat_dlug * 0.8 / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;
    float xDDŚ_LHH = - prostokat_dlug * 0.8 / 2 * cos(rad) - (75.0f + prostokat_wys) / 2 * sin(rad) + q_x;
    float yDDŚ_LHH =  prostokat_dlug * 0.8 / 2 * sin(rad) - (75.0f + prostokat_wys) / 2 * cos(rad) + q_y;


    float xŚ_LL = -prostokat_dlug * 1.2/ 2 * cos(rad) - (prostokat_wys) / 2 * sin(rad) + q_x;
    float yŚ_LL = prostokat_dlug * 1.2 / 2 * sin(rad) - (87.5f + prostokat_wys) / 2 * sin(rad) + q_y - 43.5;
    float xŚ_LP = -72 / 2 * cos(rad) - (prostokat_wys) / 2 * sin(rad) + q_x;
    float yŚ_LP = 72 / 2 * sin(rad) - (prostokat_wys) / 2 * sin(rad) + q_y - 43.5;

    float xŚ_PP = prostokat_dlug * 1.2 / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
    float yŚ_PP = -prostokat_dlug * 1.2 / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y-43.5;
    float xŚ_PL = 72 / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
    float yŚ_PL = -72 / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y-43.5;





    //render, (x,y - of the centre of elipse),rx,ry,color
    filledEllipseColor(gRenderer.get(), xŚ_LL, yŚ_LL, 21.0f, 6.0f, 0xFF00000F);
    filledEllipseColor(gRenderer.get(), xŚ_LP, yŚ_LP, 21.0f, 6.0f, 0xFF00000F);
    filledEllipseColor(gRenderer.get(), xŚ_PP, yŚ_PP, 21.0f, 6.0f, 0xFF000000); 
    filledEllipseColor(gRenderer.get(), xŚ_PL, yŚ_PL, 21.0f, 6.0f, 0xFF00000F);

    thickLineColor(gRenderer.get(), xDDŚ_LHL, yDDŚ_LHL, xDDŚ_LHH, yDDŚ_LHH, 6, 0xFFFF0000);



    thickLineColor(gRenderer.get(), x_LG, y_LG, x_PG, y_PG, 20, 0xFFFF0000);


    thickLineColor(gRenderer.get(), xDDŚ_PHL, yDDŚ_PHL, xDDŚ_PGHH, yDDŚ_PGHH, 6, 0xFFFF0000);
}


//wzór na rotacje theta
//x1 = x⋅cos(θ)−y⋅sin(θ)
//y1 = x⋅sin(θ)+y⋅cos(θ)






//x i y mają znak - dla lewego gornego rogu 

  //(lewyG)   float x1 = -half_width, y1 = -half_height;
  //(prawyG)  float x2 = half_width, y2 = -half_height;
  //(prawyD)  float x3 = half_width, y3 = half_height;
  //(lewyD)   float x4 = -half_width, y4 = half_height;






    //thickLineColor(gRenderer.get(), x_LG, y_LG, x_PG, y_PG, 0.001, 0xFF0000FF);
    //thickLineColor(gRenderer.get(), x_PG, y_PG, x_PD, y_PD, 3, 0xFF0000FF);
    //thickLineColor(gRenderer.get(), x_LD, y_LD, x_LG, y_LG, 0.001, 0xFF0000FF);



/*  float x_LG = -prostokat_dlug / 2 * cos(rad) + prostokat_wys / 2 * sin(rad) + q_x;
  float y_LG = -prostokat_dlug / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;

  float x_PG = prostokat_dlug / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
  float y_PG = - prostokat_dlug / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;*/


  // float x_PD = prostokat_dlug / 2 * cos(rad) + prostokat_wys / 2 * sin(rad) + q_x;
  // float y_PD = prostokat_dlug / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;

  // float x_LD = -prostokat_dlug / 2 * cos(rad) + prostokat_wys / 2 * sin(rad) + q_x;
  // float y_LD = -prostokat_dlug / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;



   //float xDDŚ_PHL = prostokat_dlug * 0.8 / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
   //float yDDŚ_PHL = - prostokat_dlug / 0.8 / 2 * sin(rad) - prostokat_wys / 2 * cos(rad) + q_y;

  // float xDDŚ_PGHH = prostokat_dlug * 0.8 / 2 * cos(rad) - (100.0f + prostokat_wys) / 2 * sin(rad) + q_x;
   //float yDDŚ_PGHH = - prostokat_dlug / 0.8 / 2 * sin(rad) - (100.0f+prostokat_wys) / 2 * cos(rad) + q_y;

    /*float x_PD = prostokat_dlug / 2 * cos(rad) - prostokat_wys / 2 * sin(rad) + q_x;
float y_PD = prostokat_dlug / 2 * sin(rad) + prostokat_wys / 2 * cos(rad) + q_y;

float x_LD = - prostokat_dlug / 2 * cos(rad) + prostokat_wys / 2 * sin(rad) + q_x;
float y_LD = prostokat_dlug / 2 * sin(rad)  + prostokat_wys / 2 * cos(rad) + q_y;*/

// + - 
// + +
