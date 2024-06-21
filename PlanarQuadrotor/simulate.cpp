#include "simulate.h"
#include <Eigen/Dense>

// zmiana przedzia³ki na metry
float pixelsToMetersX(int pixels, int SCREEN_WIDTH_PIXELS, float SCREEN_WIDTH_METERS) {
    return (pixels / static_cast<float>(SCREEN_WIDTH_PIXELS)) * SCREEN_WIDTH_METERS;
}

float pixelsToMetersY(int pixels, int SCREEN_HEIGHT_PIXELS, float SCREEN_HEIGHT_METERS) {
    return (pixels / static_cast<float>(SCREEN_HEIGHT_PIXELS)) * SCREEN_HEIGHT_METERS;
}

// LQR
Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A, B;
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B_discrete = Eigen::MatrixXf::Zero(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(2, 6);

    Q.diagonal() << 5, 5, 60, 1, 10, 1 / 2 / 3;
    R.row(0) << 0.1, 0.05;
    R.row(1) << 0.05, 0.1;

     
    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT) {
    if (SDL_Init(SDL_INIT_VIDEO) >= 0) {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}





int main(int argc, char* args[]) {
    const float SCREEN_WIDTH_METERS = (3.0f*1280)/720; //= 5.33
    const float SCREEN_HEIGHT_METERS = 3.0f;

    const int SCREEN_WIDTH_PIXELS = 1280;
    const int SCREEN_HEIGHT_PIXELS = 720;
    /**
 * TODO: Extend simulation
 * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
 *    [x, y, 0, 0, 0, 0]
 * 2. Update PlanarQuadrotor from simulation when goal is changed
*/
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;

    int initial_pixel_x = SCREEN_WIDTH_PIXELS / 2;
    int initial_pixel_y = SCREEN_HEIGHT_PIXELS / 2;
    float initial_x_meters = pixelsToMetersX(initial_pixel_x, SCREEN_WIDTH_PIXELS, SCREEN_WIDTH_METERS);
    float initial_y_meters = pixelsToMetersY(initial_pixel_y, SCREEN_HEIGHT_PIXELS, SCREEN_HEIGHT_METERS);
    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    initial_state << initial_x_meters, initial_y_meters, 0, 0, 0, 0;


    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor, SCREEN_WIDTH_METERS, SCREEN_HEIGHT_METERS, SCREEN_WIDTH_PIXELS, SCREEN_HEIGHT_PIXELS);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);

    const float dt = 0.001f;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH_PIXELS, SCREEN_HEIGHT_PIXELS) >= 0) {
        SDL_Event e;
        bool quit = false;
        float delay;
        int mouseX, mouseY;

        

        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit) {
            while (SDL_PollEvent(&e) != 0) {
                if (e.type == SDL_QUIT) {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN) {

                    if (e.button.button == SDL_BUTTON_LEFT) {
                        SDL_GetMouseState(&mouseX, &mouseY);
                        float mouse_x_meters = pixelsToMetersX(mouseX, SCREEN_WIDTH_PIXELS, SCREEN_WIDTH_METERS);
                        float mouse_y_meters = pixelsToMetersY(mouseY, SCREEN_WIDTH_PIXELS, SCREEN_WIDTH_METERS);
                        goal_state << mouse_x_meters,mouse_y_meters, 0, 0, 0, 0;
                        quadrotor.SetGoal(goal_state);

                    }
                }
            }

            SDL_Delay(static_cast<int>(dt * 1000));

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            //rendering step
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    SDL_Quit();
    return 0;
}
