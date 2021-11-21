//
//  main.cpp
//  EA_Controller
//
//  Created by Albert Go on 11/19/21.
//

#include <iostream>
#include <vector>
#include <math.h>
#include <numeric>
#include <list>
#include <random>
#include <algorithm>

using namespace std;

struct PointMass{
    double mass;
    vector<float> position; // {x, y, z}
    vector<float> velocity; // {v_x, v_y, v_z}
    vector<float> acceleration; // {a_x, a_y, a_z}
    vector<float> forces; // {f_x, f_y, f_z}
    int ID; //index of where a particular mass lies in the robot.masses vectorOoops
};

struct Spring{
    float L0; // resting length
    float L; // current length
    float k; // spring constant
    int m0; // connected to which PointMass
    int m1; // connected to which PointMass
    float original_L0;
    int ID; //the index of where a particular string lies in the robot.springs vector
};

struct Cube{
    vector<PointMass> masses;
    vector<Spring> springs;
    vector<int> joinedCubes;
    vector<int> otherFaces; //faces of other cubes that are joined to it
    vector<int> joinedFaces; //faces of the cube that are joined to other cubes
    vector<int> massIDs; //where the verteces of the cube correspond to the Robot.masses vector
    vector<int> springIDs; //where the springs of the cube correspond to the Robot.springs vector
};

struct Robot{
    vector<PointMass> masses; //vector of masses that make up the robot
    vector<Spring> springs; //vector of springs that make up the robot
    vector<int> cubes;
    vector<Cube> all_cubes;
};

struct Equation{
    float k;
    float a;
    float w;
    float c;
};

struct Controller{
    vector<Equation> motor;
    vector<float> start;
    vector<float> end;
    float fitness;
};

const double g = -9.81; //acceleration due to gravity
const double b = 1.0; //damping (optional) Note: no damping means your cube will bounce forever
const float spring_constant = 5000.0f; //this worked best for me given my dt and mass of each PointMass
const float mu_s = 0.74; //coefficient of static friction
const float mu_k = 0.57; //coefficient of kinetic friction
float T = 0.0;
float dt = 0.0001;
bool breathing = true;

const int cut_point1 = 6;
const int cut_point2 = 15;

vector<int> face0 = {0, 1, 2, 3}; //face 0 (bottom face) corresponds with these cube vertices; only connects with face 5
vector<int> face1 = {0, 3, 4, 7}; //face 1(front face) corresponds with these cube vertices; only connects with face 3
vector<int> face2 = {0, 1, 4, 5}; //face 2 (left face) corresponds with these cube vertices; only connects with face 4
vector<int> face3 = {1, 2, 5, 6}; //face 3 (back face) corrresponds with these cube vertices; only connects with face 1
vector<int> face4 = {3, 2, 7, 6}; //face 4 (right face) corresponds with these cube vertices; only conncects with face 2
vector<int> face5 = {4, 5, 6, 7}; //face 5 (top face) corresponds with these cube vertices; only connects with face 0

vector<int> face0_springs = {0, 1, 2, 3, 4, 5}; //face 0 (bottom face) corresponds with these cube springs; only connects with face 5
vector<int> face1_springs = {3, 6, 9, 10, 11, 21}; //face 1 (front face) corresponds with these cube springs; only connects with face 3
vector<int> face2_springs = {0, 6, 7, 12, 13, 18}; //face 2 (left face) corresponds with these cube springs; only connects with face 4
vector<int> face3_springs = {1, 7, 8, 14, 15, 19}; //face 3 (back face) corresponds with these cube springs; only connects with face 1
vector<int> face4_springs = {2, 9, 8, 17, 16, 20}; //face 4 (right face) corresponds with these cube springs; only connects with face 2
vector<int> face5_springs = {18, 19, 20, 21, 22, 23}; // face 5 (top face) corresponds with these cube springs; only connects with face 0

vector<float> const_k = {1000, 5000, 5000, 10000};
vector<float> const_a = {0.1, 0.12, 0.15};
vector<float> const_w = {M_PI, 2*M_PI};
vector<float> const_c = {0, M_PI};

void initialize_masses(vector<PointMass> &masses);
void initialize_springs(vector<Spring> &springs);
void apply_force(vector<PointMass> &masses);
void update_pos_vel_acc(Robot &robot);
void update_forces(Robot &robot);
void reset_forces(Robot &robot);
void update_breathing(Robot &robot, Controller &control);
void initialize_robot(Robot &robot);
void join_cubes(Cube &cube1, Cube &cube2, int cube1_index, int cube2_index, vector<PointMass> &masses, vector<Spring> &springs, int combine1, int combine2, Cube &cube3);
void initialize_cube(Cube &cube);
void get_population(vector<Controller> &population, Robot &robot);
void create_equation(Controller &control);
float determine_fitness(Controller &control, Robot robot);
bool compareByFitness(const Controller &control1, const Controller &control2);
void mutate(Controller &offspring);
void breed(vector<Controller> &new_population, Controller control1, Controller control2, Robot &robot);
void replenish_population(vector<Controller> &new_set, Robot &robot);


int main(int argc, const char * argv[]) {
    // insert code here...
    srand( static_cast<unsigned int>(time(0)));
    std::cout << "Evolving controller for robot..." << endl;
    
    Robot robot;
    initialize_robot(robot);
    
    cout<< robot.masses.size() << endl;
    cout<< robot.springs.size() << endl;
    cout<< robot.all_cubes.size() << endl;
    
    vector<Controller> population;
    vector<Controller> major_league;
    vector<Controller> update_major;
    
    get_population(population, robot);
    sort(population.begin(), population.end(), compareByFitness);
    cout<< "Initialized Population" << endl;

    int evaluations = 0;
    
    
    // render loop
    while(evaluations < 10000)
    {
        vector<Controller> new_population;
        vector<Controller> new_major;
        cout << population.size();
        cout << ", ";
        cout << major_league.size() << endl;
        for (int i=0; i<population.size(); i++){
            int parent2 = rand() % 50;
            if(parent2 == i){
                bool same = true;
                while(same){
                    parent2 = rand() % 50;
                    if(parent2 != i){
                        same = false;
                    }
                }
            }
            breed(new_population, population[i], population[parent2], robot);
            if (i < major_league.size() && major_league.size() > 0){
                int parent2_m2 = rand() % 25;
                if(parent2_m2 == i){
                    bool same = true;
                    while(same){
                        parent2_m2 = rand() % 25;
                        if(parent2_m2 != i){
                            same = false;
                        }
                    }
                }
                breed(new_major, major_league[i], major_league[parent2_m2], robot);
            }
        }
        population = new_population;
        major_league = new_major;
        
        sort(population.begin(), population.end(), compareByFitness);
        sort(major_league.begin(), major_league.end(), compareByFitness);
        
        
        evaluations += 1;
        
        if (evaluations % 10 == 0){
            if (major_league.size() > 0){
                cout << "Major League update: ";
                cout << major_league[0].fitness << endl;
                
                cout << "< ";
                for (int j=0; j<major_league[0].motor.size(); j++){
                    cout << "[";
                    cout << major_league[0].motor[j].k;
                    cout << ", ";
                    cout << major_league[0].motor[j].a;
                    cout << ", ";
                    cout << major_league[0].motor[j].w;
                    cout << ", ";
                    cout << major_league[0].motor[j].c;
                    cout << "]";
                    cout << ", ";
                }
                cout << "> " << endl;
            }
            else{
                cout << "Status update: ";
                cout << population[0].fitness << endl;
                cout << "Updating population..." << endl;
                
                cout << "< ";
                for (int j=0; j<population[0].motor.size(); j++){
                    cout << "[";
                    cout << population[0].motor[j].k;
                    cout << ", ";
                    cout << population[0].motor[j].a;
                    cout << ", ";
                    cout << population[0].motor[j].w;
                    cout << ", ";
                    cout << population[0].motor[j].c;
                    cout << "]";
                    cout << ", ";
                }
                cout << "> " << endl;
            }
            
            
            if (major_league.size() > 0){
                major_league.erase(major_league.begin()+25, major_league.end());
            }
            update_major = {population.begin(), population.begin()+50};
            major_league.insert(major_league.end(), update_major.begin(), update_major.end());
            
            population.erase(population.begin(), population.begin()+50);
            
            vector<Controller> new_set;
            replenish_population(new_set, robot);
            
            population.insert(population.end(), new_set.begin(), new_set.end());
            
            sort(population.begin(), population.end(), compareByFitness);
            sort(major_league.begin(), major_league.end(), compareByFitness);
        }
        if (major_league.size() > 0){
            cout << "Major League update: ";
            cout << major_league[0].fitness << endl;
            
            cout << "< ";
            for (int j=0; j<major_league[0].motor.size(); j++){
                cout << "[";
                cout << major_league[0].motor[j].k;
                cout << ", ";
                cout << major_league[0].motor[j].a;
                cout << ", ";
                cout << major_league[0].motor[j].w;
                cout << ", ";
                cout << major_league[0].motor[j].c;
                cout << "]";
                cout << ", ";
            }
            cout << "> " << endl;
        }
        else{
            cout << "Status update: ";
            cout << population[0].fitness << endl;
            cout << "Updating population..." << endl;
            
            cout << "< ";
            for (int j=0; j<population[0].motor.size(); j++){
                cout << "[";
                cout << population[0].motor[j].k;
                cout << ", ";
                cout << population[0].motor[j].a;
                cout << ", ";
                cout << population[0].motor[j].w;
                cout << ", ";
                cout << population[0].motor[j].c;
                cout << "]";
                cout << ", ";
            }
            cout << "> " << endl;
        }
        
        cout << evaluations << endl;
        
        
    }
    
    return 0;
}

void get_population(vector<Controller> &population, Robot &robot){
    int individuals = 0;
    
    while (individuals < 100) {
        cout << "New Individual" << endl;
        Controller control;
        create_equation(control);
        
        float x_center = 0;
        float y_center = 0;
        float z_center = 0;
        for (int m=0; m<robot.masses.size(); m++){
            x_center += robot.masses[m].position[0];
            y_center += robot.masses[m].position[1];
            z_center += robot.masses[m].position[2];
        }
        
        x_center = x_center/robot.masses.size();
        y_center = y_center/robot.masses.size();
        z_center = z_center/robot.masses.size();
        
        cout << "Center = ";
        cout << x_center;
        cout << ", ";
        cout << y_center;
        cout << ", ";
        cout << z_center << endl;
        
        control.start = {x_center, y_center, z_center};
        
        control.fitness = determine_fitness(control, robot);
        
        cout << "Fitness = ";
        cout << control.fitness << endl;
        
//        cout << "< ";
//        for (int j=0; j<control.motor.size(); j++){
//            cout << "[";
//            cout << control.motor[j].k;
//            cout << ", ";
//            cout << control.motor[j].a;
//            cout << ", ";
//            cout << control.motor[j].w;
//            cout << ", ";
//            cout << control.motor[j].c;
//            cout << "]";
//            cout << ", ";
//        }
//        cout << "> " << endl;
        
        population.push_back(control);
        individuals += 1;
    }
}

void replenish_population(vector<Controller> &new_set, Robot &robot){
    int individuals = 0;
    
    while (individuals < 50) {
        cout << "Replenishing Population..." << endl;
        Controller control;
        create_equation(control);
        
        float x_center = 0;
        float y_center = 0;
        float z_center = 0;
        for (int m=0; m<robot.masses.size(); m++){
            x_center += robot.masses[m].position[0];
            y_center += robot.masses[m].position[1];
            z_center += robot.masses[m].position[2];
        }
        
        x_center = x_center/robot.masses.size();
        y_center = y_center/robot.masses.size();
        z_center = z_center/robot.masses.size();
        
//        cout << "Center = ";
//        cout << x_center;
//        cout << ", ";
//        cout << y_center;
//        cout << ", ";
//        cout << z_center << endl;
        
        control.start = {x_center, y_center, z_center};
        
        control.fitness = determine_fitness(control, robot);
        
        cout << "Fitness = ";
        cout << control.fitness << endl;
        
        new_set.push_back(control);
        individuals += 1;
    }
}

void create_equation(Controller &control){
    for (int i=0; i<22; i++){
        Equation eqn;
        int rand1 = rand() % 4;
        int rand2 = rand() % 3;
        int rand3 = rand() % 2;
        int rand4 = rand() % 2;
        
        eqn.k = const_k[rand1];
        if (rand1 == 0){
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (rand1 == 3){
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else{
            eqn.a = const_a[rand2];
            eqn.w = const_w[rand3];
            eqn.c = const_c[rand4];
        }
        
        control.motor.push_back(eqn);
    }
}

float determine_fitness(Controller &control, Robot robot){
    float displacement = 0;
    int runs = 0;
    T = 0;
    
    while (runs < 300){
        
        //Let's test the controller
        //-------------------------------------
        for (int k=0; k<50; k++){
            T = T + dt; //update time that has passed
            if (breathing) {
                update_breathing(robot, control);
            }

            update_forces(robot);
            update_pos_vel_acc(robot);
            
            reset_forces(robot);
            
        }
        //-------------------------------------
        
        runs += 1;
    }
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    for (int m=0; m<robot.masses.size(); m++){
        x_center += robot.masses[m].position[0];
        y_center += robot.masses[m].position[1];
        z_center += robot.masses[m].position[2];
    }
    
    x_center = x_center/robot.masses.size();
    y_center = y_center/robot.masses.size();
    z_center = z_center/robot.masses.size();
    
    control.end = {x_center, y_center, z_center};
    
    displacement = sqrt(pow(control.end[0]-control.start[0], 2) + pow(control.end[1]-control.start[1], 2));
    
    return displacement;
}

void breed(vector<Controller> &new_population, Controller control1, Controller control2, Robot &robot){
    Controller offspring;
    
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    for (int m=0; m<robot.masses.size(); m++){
        x_center += robot.masses[m].position[0];
        y_center += robot.masses[m].position[1];
        z_center += robot.masses[m].position[2];
    }
    
    x_center = x_center/robot.masses.size();
    y_center = y_center/robot.masses.size();
    z_center = z_center/robot.masses.size();
    
    offspring.start = {x_center, y_center, z_center};
    
    bool recomb = false;
    for (int i=0; i<22; i++){
        if (i==cut_point1){
            recomb = true;
        }
        else if (i==cut_point2){
            recomb = false;
        }
        if (recomb){
            offspring.motor.push_back(control2.motor[i]);
        }
        else{
            offspring.motor.push_back(control1.motor[i]);
        }
    }
    
    
    int rand1 = rand() % 100;
    
    if (rand1 < 50){
        mutate(offspring);
    }
    
//    cout << "Offspring size: ";
//    cout << offspring.motor.size() << endl;
    
    offspring.fitness = determine_fitness(offspring, robot);
    
    if (offspring.fitness > control1.fitness){
        new_population.push_back(offspring);
    }
    else{
        new_population.push_back(control1);
    }
    
}

void mutate(Controller &offspring){
    int rand_num = rand() % 22;
    int rand_num2 = rand() %22;
    if(rand_num2 == rand_num){
        bool same = true;
        while(same){
            rand_num2 = rand() % 22;
            if(rand_num2 != rand_num){
                same = false;
            }
        }
    }
    
    iter_swap(offspring.motor.begin()+rand_num, offspring.motor.begin()+rand_num2);
}

void update_breathing(Robot &robot, Controller &control){
//    for (int i=0; i<28; i++){
//        int ind = robot.all_cubes[0].springIDs[i];
//        robot.springs[ind].L0 = robot.all_cubes[0].springs[i].original_L0 + 0.05f*sin(T+1);
//    }
    for (int i=0; i<robot.all_cubes.size(); i++){
        int ind0 = robot.all_cubes[i].springIDs[0];
        int ind1 = robot.all_cubes[i].springIDs[1];
        int ind2 = robot.all_cubes[i].springIDs[2];
        int ind3 = robot.all_cubes[i].springIDs[3];
        int ind4 = robot.all_cubes[i].springIDs[4];
        int ind5 = robot.all_cubes[i].springIDs[5];
        int ind6 = robot.all_cubes[i].springIDs[6];
        int ind7 = robot.all_cubes[i].springIDs[7];
        int ind8 = robot.all_cubes[i].springIDs[8];
        int ind9 = robot.all_cubes[i].springIDs[9];
        int ind10 = robot.all_cubes[i].springIDs[10];
        int ind11 = robot.all_cubes[i].springIDs[11];
        int ind12 = robot.all_cubes[i].springIDs[12];
        int ind13 = robot.all_cubes[i].springIDs[13];
        int ind14 = robot.all_cubes[i].springIDs[14];
        int ind15 = robot.all_cubes[i].springIDs[15];
        int ind16 = robot.all_cubes[i].springIDs[16];
        int ind17 = robot.all_cubes[i].springIDs[17];
        int ind18 = robot.all_cubes[i].springIDs[18];
        int ind19 = robot.all_cubes[i].springIDs[19];
        int ind20 = robot.all_cubes[i].springIDs[20];
        int ind21 = robot.all_cubes[i].springIDs[21];
        int ind22 = robot.all_cubes[i].springIDs[22];
        int ind23 = robot.all_cubes[i].springIDs[23];
        int ind24 = robot.all_cubes[i].springIDs[24];
        int ind25 = robot.all_cubes[i].springIDs[25];
        int ind26 = robot.all_cubes[i].springIDs[26];
        int ind27 = robot.all_cubes[i].springIDs[27];
        
        float k = control.motor[i].k;
        float a = control.motor[i].a;
        float w = control.motor[i].w;
        float c = control.motor[i].c;
        
        robot.springs[ind0].L0 = robot.all_cubes[i].springs[0].original_L0 + a*sin(w*T+c);
        robot.springs[ind1].L0 = robot.all_cubes[i].springs[1].original_L0 + a*sin(w*T+c);
        robot.springs[ind2].L0 = robot.all_cubes[i].springs[2].original_L0 + a*sin(w*T+c);
        robot.springs[ind3].L0 = robot.all_cubes[i].springs[3].original_L0 + a*sin(w*T+c);
        robot.springs[ind4].L0 = robot.all_cubes[i].springs[4].original_L0 + a*sin(w*T+c);
        robot.springs[ind5].L0 = robot.all_cubes[i].springs[5].original_L0 + a*sin(w*T+c);
        robot.springs[ind6].L0 = robot.all_cubes[i].springs[6].original_L0 + a*sin(w*T+c);
        robot.springs[ind7].L0 = robot.all_cubes[i].springs[7].original_L0 + a*sin(w*T+c);
        robot.springs[ind8].L0 = robot.all_cubes[i].springs[8].original_L0 + a*sin(w*T+c);
        robot.springs[ind9].L0 = robot.all_cubes[i].springs[9].original_L0 + a*sin(w*T+c);
        robot.springs[ind10].L0 = robot.all_cubes[i].springs[10].original_L0 + a*sin(w*T+c);
        robot.springs[ind11].L0 = robot.all_cubes[i].springs[11].original_L0 + a*sin(w*T+c);
        robot.springs[ind12].L0 = robot.all_cubes[i].springs[12].original_L0 + a*sin(w*T+c);
        robot.springs[ind13].L0 = robot.all_cubes[i].springs[13].original_L0 + a*sin(w*T+c);
        robot.springs[ind14].L0 = robot.all_cubes[i].springs[14].original_L0 + a*sin(w*T+c);
        robot.springs[ind15].L0 = robot.all_cubes[i].springs[15].original_L0 + a*sin(w*T+c);
        robot.springs[ind16].L0 = robot.all_cubes[i].springs[16].original_L0 + a*sin(w*T+c);
        robot.springs[ind17].L0 = robot.all_cubes[i].springs[17].original_L0 + a*sin(w*T+c);
        robot.springs[ind18].L0 = robot.all_cubes[i].springs[18].original_L0 + a*sin(w*T+c);
        robot.springs[ind19].L0 = robot.all_cubes[i].springs[19].original_L0 + a*sin(w*T+c);
        robot.springs[ind20].L0 = robot.all_cubes[i].springs[20].original_L0 + a*sin(w*T+c);
        robot.springs[ind21].L0 = robot.all_cubes[i].springs[21].original_L0 + a*sin(w*T+c);
        robot.springs[ind22].L0 = robot.all_cubes[i].springs[22].original_L0 + a*sin(w*T+c);
        robot.springs[ind23].L0 = robot.all_cubes[i].springs[23].original_L0 + a*sin(w*T+c);
        robot.springs[ind24].L0 = robot.all_cubes[i].springs[24].original_L0 + a*sin(w*T+c);
        robot.springs[ind25].L0 = robot.all_cubes[i].springs[25].original_L0 + a*sin(w*T+c);
        robot.springs[ind26].L0 = robot.all_cubes[i].springs[26].original_L0 + a*sin(w*T+c);
        robot.springs[ind27].L0 = robot.all_cubes[i].springs[27].original_L0 + a*sin(w*T+c);
        
        robot.springs[ind0].k = k;
        robot.springs[ind1].k = k;
        robot.springs[ind2].k = k;
        robot.springs[ind3].k = k;
        robot.springs[ind4].k = k;
        robot.springs[ind5].k = k;
        robot.springs[ind6].k = k;
        robot.springs[ind7].k = k;
        robot.springs[ind8].k = k;
        robot.springs[ind9].k = k;
        robot.springs[ind10].k = k;
        robot.springs[ind11].k = k;
        robot.springs[ind12].k = k;
        robot.springs[ind13].k = k;
        robot.springs[ind14].k = k;
        robot.springs[ind15].k = k;
        robot.springs[ind16].k = k;
        robot.springs[ind17].k = k;
        robot.springs[ind18].k = k;
        robot.springs[ind19].k = k;
        robot.springs[ind20].k = k;
        robot.springs[ind21].k = k;
        robot.springs[ind22].k = k;
        robot.springs[ind23].k = k;
        robot.springs[ind24].k = k;
        robot.springs[ind25].k = k;
        robot.springs[ind26].k = k;
        robot.springs[ind27].k = k;
    }
}

void update_pos_vel_acc(Robot &robot){
    
    for (int i=0; i<robot.masses.size(); i++){
        float acc_x = robot.masses[i].forces[0]/robot.masses[i].mass;
        float acc_y = robot.masses[i].forces[1]/robot.masses[i].mass;
        float acc_z = robot.masses[i].forces[2]/robot.masses[i].mass;

        robot.masses[i].acceleration[0] = acc_x;
        robot.masses[i].acceleration[1] = acc_y;
        robot.masses[i].acceleration[2] = acc_z;
        
        float vel_x = acc_x*dt + robot.masses[i].velocity[0];
        float vel_y = acc_y*dt + robot.masses[i].velocity[1];
        float vel_z = acc_z*dt + robot.masses[i].velocity[2];
        
        
        robot.masses[i].velocity[0] = vel_x*b;
        robot.masses[i].velocity[1] = vel_y*b;
        robot.masses[i].velocity[2] = vel_z*b;
        
        float pos_x = (vel_x*dt) + robot.masses[i].position[0];
        float pos_y = (vel_y*dt) + robot.masses[i].position[1];
        float pos_z = (vel_z*dt) + robot.masses[i].position[2];
        
        robot.masses[i].position[0] = pos_x;
        robot.masses[i].position[1] = pos_y;
        robot.masses[i].position[2] = pos_z;
    }
    
    for (int j=0; j<robot.all_cubes.size(); j++){
        
        int ind0 = robot.all_cubes[j].massIDs[0];
        int ind1 = robot.all_cubes[j].massIDs[1];
        int ind2 = robot.all_cubes[j].massIDs[2];
        int ind3 = robot.all_cubes[j].massIDs[3];
        int ind4 = robot.all_cubes[j].massIDs[4];
        int ind5 = robot.all_cubes[j].massIDs[5];
        int ind6 = robot.all_cubes[j].massIDs[6];
        int ind7 = robot.all_cubes[j].massIDs[7];
        
        robot.all_cubes[j].masses[0].position[0] = robot.masses[ind0].position[0];
        robot.all_cubes[j].masses[0].position[1] = robot.masses[ind0].position[1];
        robot.all_cubes[j].masses[0].position[2] = robot.masses[ind0].position[2];
        
        robot.all_cubes[j].masses[1].position[0] = robot.masses[ind1].position[0];
        robot.all_cubes[j].masses[1].position[1] = robot.masses[ind1].position[1];
        robot.all_cubes[j].masses[1].position[2] = robot.masses[ind1].position[2];
        
        robot.all_cubes[j].masses[2].position[0] = robot.masses[ind2].position[0];
        robot.all_cubes[j].masses[2].position[1] = robot.masses[ind2].position[1];
        robot.all_cubes[j].masses[2].position[2] = robot.masses[ind2].position[2];
        
        robot.all_cubes[j].masses[3].position[0] = robot.masses[ind3].position[0];
        robot.all_cubes[j].masses[3].position[1] = robot.masses[ind3].position[1];
        robot.all_cubes[j].masses[3].position[2] = robot.masses[ind3].position[2];
        
        robot.all_cubes[j].masses[4].position[0] = robot.masses[ind4].position[0];
        robot.all_cubes[j].masses[4].position[1] = robot.masses[ind4].position[1];
        robot.all_cubes[j].masses[4].position[2] = robot.masses[ind4].position[2];
        
        robot.all_cubes[j].masses[5].position[0] = robot.masses[ind5].position[0];
        robot.all_cubes[j].masses[5].position[1] = robot.masses[ind5].position[1];
        robot.all_cubes[j].masses[5].position[2] = robot.masses[ind5].position[2];
        
        robot.all_cubes[j].masses[6].position[0] = robot.masses[ind6].position[0];
        robot.all_cubes[j].masses[6].position[1] = robot.masses[ind6].position[1];
        robot.all_cubes[j].masses[6].position[2] = robot.masses[ind6].position[2];
        
        robot.all_cubes[j].masses[7].position[0] = robot.masses[ind7].position[0];
        robot.all_cubes[j].masses[7].position[1] = robot.masses[ind7].position[1];
        robot.all_cubes[j].masses[7].position[2] = robot.masses[ind7].position[2];
    }
}

void reset_forces(Robot &robot){
    for(int i=0; i<robot.masses.size(); i++){
        robot.masses[i].forces = {0.0f, 0.0f, 0.0f};
    }
}

void update_forces(Robot &robot){
    
    for (int i=0; i<robot.springs.size(); i++){

        int p0 = robot.springs[i].m0;
        int p1 = robot.springs[i].m1;

        vector<float> pos0 = robot.masses[p0].position;
        vector<float> pos1 = robot.masses[p1].position;

        float spring_length = sqrt(pow(pos1[0]-pos0[0], 2) + pow(pos1[1]-pos0[1], 2) + pow(pos1[2]-pos0[2], 2));

        robot.springs[i].L = spring_length;
        float force = -robot.springs[i].k*(spring_length-robot.springs[i].L0);

        float x_univ = (pos0[0]-pos1[0])/spring_length;
        float y_univ = (pos0[1]-pos1[1])/spring_length;
        float z_univ = (pos0[2]-pos1[2])/spring_length;
        vector<float> force_unit_dir_2_1 = {x_univ,y_univ,z_univ};
        vector<float> force_unit_dir_1_2 = {-x_univ,-y_univ,-z_univ};

        for (int n = 0; n < 3; n++) {
            robot.masses[p0].forces[n] =  robot.masses[p0].forces[n] + force * force_unit_dir_2_1[n];
            robot.masses[p1].forces[n] =  robot.masses[p1].forces[n] + force * force_unit_dir_1_2[n];
        }
    }
    
    for (int j=0; j<robot.masses.size(); j++){
        robot.masses[j].forces[2] = robot.masses[j].forces[2] + robot.masses[j].mass*g;
        
        if (robot.masses[j].position[2] < 0){
            robot.masses[j].forces[2] = -robot.masses[j].position[2]*100000.0f;
        }
        
        float F_n = robot.masses[j].mass*g;
        
        float F_h = sqrt(pow(robot.masses[j].forces[0], 2) + pow(robot.masses[j].forces[1], 2));
        
        if (F_h < -F_n*mu_s){
            robot.masses[j].forces[0] = 0;
            robot.masses[j].forces[1] = 0;
        }
        
        if (F_n < 0){
            if (F_h >= -F_n*mu_s){
                if (robot.masses[j].forces[0] > 0){
                    robot.masses[j].forces[0] = robot.masses[j].forces[0] + mu_k*F_n;
                }
                else{
                    robot.masses[j].forces[0] = robot.masses[j].forces[0] - mu_k*F_n;
                }
                if (robot.masses[j].forces[1] > 0){
                    robot.masses[j].forces[1] = robot.masses[j].forces[1] + mu_k*F_n;
                }
                else{
                    robot.masses[j].forces[1] = robot.masses[j].forces[1] - mu_k*F_n;
                }
            }
        }
    }
}

void initialize_robot(Robot &robot){
    vector<PointMass> masses; //initializes the vector of masses that make up the robot
    vector<Spring> springs; //initializes the vector of springs that make up the robot
    vector<int> cubes;
    vector<Cube> all_cubes; //initializes all the cubes that will make up this robot
    for (int i=0; i<22; i++){
        Cube cube; //define a cube
        initialize_cube(cube); //initialize the cube
        cubes.push_back(i);
        if (i==0){
            //for the first cube, you can add everything
            for (int j=0; j<28; j++){
                cube.springs[j].ID = j;
                cube.springIDs.push_back(j);
                springs.push_back(cube.springs[j]);
            }
            for (int k=0; k<8; k++){
                cube.masses[k].ID = k;
                cube.massIDs.push_back(k);
                masses.push_back(cube.masses[k]);
            }
        }
        else{
            //for every subsequent cube, you need to join it to the existing masses and springs
            if (i<3){
                //TODO: Need to build on the top face of the cube; starts with cube 0
                join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 5, 0, cube);
            }
            else if (i>=3 && i<7){
                //TODO: Need to build on the front face of the cube; starts with building on cube 2 front face
                join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 1, 3, cube);
            }
            else if (i>=7 && i<9){
                //TODO: Need to build on the bottom face of the cube; starts with building on cube 6 bottom face
                join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 0, 5, cube);
            }
            else if (i>=9 && i<12){
                //TODO: Go back to cube 3 and build on its right face
                if (i == 9){
                    join_cubes(all_cubes[3], cube, 3, i, masses, springs, 4, 2, cube);
                }
                else{
                    join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 4, 2, cube);
                }
            }
            else if (i == 12){
                //TODO: Build on the 11th cube's back face
                join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 3, 1, cube);
            }
            else if (i>12 && i <15){
                //TODO: Build on the 12th cube's bottom face
                join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 0, 5, cube);
            }
            else if (i>14 && i<18){
                //TODO: Go back to cube 11 and build on its front face
                if (i==15){
                    join_cubes(all_cubes[11], cube, 11, i, masses, springs, 1, 3, cube);
                }
                else{
                    join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 1, 3, cube);
                }
            }
            else if (i>17 && i<20){
                //TODO: Build on cube 17 and build on its bottom face
                join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 0, 5, cube);
            }
            else{
                //TODO: Go back to cube 5 and build on its right face
                if (i==20){
                    join_cubes(all_cubes[5], cube, 5, i, masses, springs, 4, 2, cube);
                }
                else{
                    join_cubes(all_cubes.back(), cube, all_cubes.size()-1, i, masses, springs, 4, 2, all_cubes[16]);
                }
            }
        }
        all_cubes.push_back(cube);
    }
    robot.masses = masses;
    robot.springs = springs;
    robot.all_cubes = all_cubes;
//    cout<< "Hello" << endl;
    
//    for (int j=0; j<robot.springs.size(); j++){
//        cout << j;
//        cout << ", ";
//        cout << robot.springs[j].m0;
//        cout << ", ";
//        cout << robot.springs[j].m1 << endl;
//    }
}

void join_cubes(Cube &cube1, Cube &cube2, int cube1_index, int cube2_index, vector<PointMass> &masses, vector<Spring> &springs, int combine1, int combine2, Cube &cube3){

    vector<int> map1;
    vector<int> map2;
    vector<int> map1_springs;
    vector<int> map2_springs;
    if (combine1 == 0){
        map1 = face0;
        map2 = face5;
        
        map1_springs = face0_springs;
        map2_springs = face5_springs;
    }
    else if (combine1 == 5){
        map1 = face5;
        map2 = face0;
        
        map1_springs = face5_springs;
        map2_springs = face0_springs;
    }
    else if (combine1 == 1){
        map1 = face1;
        map2 = face3;
        
        map1_springs = face1_springs;
        map2_springs = face3_springs;
    }
    else if (combine1 == 3){
        map1 = face3;
        map2 = face1;
        
        map1_springs = face3_springs;
        map2_springs = face1_springs;
    }
    else{
        map1 = face4;
        map2 = face2;
        
        map1_springs = face4_springs;
        map2_springs = face2_springs;
    }
    
    cube1.joinedCubes.push_back(cube2_index);
    cube1.joinedFaces.push_back(combine1);
    cube1.otherFaces.push_back(combine2);
    
    cube2.joinedCubes.push_back(cube1_index);
    cube2.joinedFaces.push_back(combine2);
    cube2.otherFaces.push_back(combine1);
    
    //find where the second cube needs to join the first cube
    float x_disp = cube2.masses[map2[0]].position[0]-cube1.masses[map1[0]].position[0]; //x displacement
    float y_disp = cube2.masses[map2[0]].position[1]-cube1.masses[map1[0]].position[1]; //y displacement
    float z_disp = cube2.masses[map2[0]].position[2]-cube1.masses[map1[0]].position[2]; //z displacement
    
    for (int i=0; i<8; i++){
        //shift cube 2 over
        cube2.masses[i].position[0] -= x_disp;
        cube2.masses[i].position[1] -= y_disp;
        cube2.masses[i].position[2] -= z_disp;
    }
    
    if (cube2_index == 21){
        for (int j=0; j<8; j++){
            if (count(map2.begin(), map2.end(), j)){
                //if the cube 2 vertex is on face 2, then you need to make sure that the ID is set to that vertex of the first cube
                int itr = find(map2.begin(), map2.end(), j)-map2.begin();
                cube2.masses[j].ID = cube1.masses[map1[itr]].ID; //set the mass ID to its position in the masses vector of the robot
                cube2.massIDs.push_back(cube1.masses[map1[itr]].ID); //add the mass IDs to the list of masses that correspond to cube2
            }
            else{
                // if the vertex is not part of face 2 then you can add it to the big vector of masses and make the ID the index of where it is in the big vector of masses
                int itr = find(map1.begin(), map1.end(), j)-map1.begin();
                cube2.masses[j].ID = cube3.masses[map2[itr]].ID;
                cube2.massIDs.push_back(cube3.masses[map2[itr]].ID);
//                masses.push_back(cube2.masses[j]);
            }
//            cout << "MASSES" << endl;
//            cout << j;
//            cout << ", ";
//            cout << cube2.masses[j].ID << endl;
//            cout << "--------" << endl;
        }
            
        for (int k=0; k<28; k++){
            int p0 = cube2.springs[k].m0;
            int p1 = cube2.springs[k].m1;
            
            if ((count(map2.begin(), map2.end(), p0)) && (count(map2.begin(), map2.end(), p1))){
                int itr = find(map2_springs.begin(), map2_springs.end(), k)-map2_springs.begin();
                int itr2 = find(map2.begin(), map2.end(), p0)-map2.begin();
                int itr3 = find(map2.begin(), map2.end(), p1)-map2.begin();
                
                cube2.springs[k].m0 = cube1.masses[map1[itr2]].ID;
                cube2.springs[k].m1 = cube1.masses[map1[itr3]].ID;
                cube2.springs[k].ID = cube1.springs[map1_springs[itr]].ID;
                cube2.springIDs.push_back(cube1.springs[map1_springs[itr]].ID);
            }
            else if ((count(map1.begin(), map1.end(), p0)) && (count(map1.begin(), map1.end(), p1))){
                int itr = find(map1_springs.begin(), map1_springs.end(), k)-map1_springs.begin();
                int itr2 = find(map1.begin(), map1.end(), p0)-map1.begin();
                int itr3 = find(map1.begin(), map1.end(), p1)-map1.begin();
                
                cube2.springs[k].m0 = cube3.masses[map2[itr2]].ID;
                cube2.springs[k].m1 = cube3.masses[map2[itr3]].ID;
                cube2.springs[k].ID = cube3.springs[map2_springs[itr]].ID;
                cube2.springIDs.push_back(cube3.springs[map2_springs[itr]].ID);
            }
            else if (find(map2.begin(), map2.end(), p0) != map2.end()){
                int itr = find(map2.begin(), map2.end(), p0)-map2.begin();
                int itr2 = find(map1.begin(), map1.end(), p1)-map1.begin();
                
                cube2.springs[k].m0 = cube1.masses[map1[itr]].ID;
                cube2.springs[k].m1 = cube3.masses[map2[itr2]].ID;
                cube2.springs[k].ID = springs.size();
                cube2.springIDs.push_back(springs.size());
                springs.push_back(cube2.springs[k]);
            }
            else if (find(map2.begin(), map2.end(), p1) != map2.end()){
                int itr = find(map2.begin(), map2.end(), p1)-map2.begin();
                int itr2 = find(map1.begin(), map1.end(), p0)-map1.begin();
                
                cube2.springs[k].m0 = cube3.masses[map2[itr2]].ID;
                cube2.springs[k].m1 = cube1.masses[map1[itr]].ID;
                cube2.springs[k].ID = springs.size();
                cube2.springIDs.push_back(springs.size());
                springs.push_back(cube2.springs[k]);
            }
//            else{
//                cube2.springs[k].m0 = cube2.masses[p0].ID;
//                cube2.springs[k].m1 = cube2.masses[p1].ID;
//                cube2.springs[k].ID = springs.size();
//                cube2.springIDs.push_back(springs.size());
//                springs.push_back(cube2.springs[k]);
//            }
        }
    }
    else{
        //joining cube 2 on the right face of the first cube; this means the left face of cube 2 and the right face of cube 1 will be joined
        for (int j=0; j<8; j++){
            if (count(map2.begin(), map2.end(), j)){
                //if the cube 2 vertex is on face 2, then you need to make sure that the ID is set to that vertex of the first cube
                int itr = find(map2.begin(), map2.end(), j)-map2.begin();
                cube2.masses[j].ID = cube1.masses[map1[itr]].ID; //set the mass ID to its position in the masses vector of the robot
                cube2.massIDs.push_back(cube1.masses[map1[itr]].ID); //add the mass IDs to the list of masses that correspond to cube2
            }
            else{
                // if the vertex is not part of face 2 then you can add it to the big vector of masses and make the ID the index of where it is in the big vector of masses
                cube2.masses[j].ID = masses.size();
                cube2.massIDs.push_back(masses.size());
                masses.push_back(cube2.masses[j]);
            }
//            cout << "MASSES" << endl;
//            cout << j;
//            cout << ", ";
//            cout << cube2.masses[j].ID << endl;
//            cout << "--------" << endl;
        }
        
        for (int k=0; k<28; k++){
            int p0 = cube2.springs[k].m0;
            int p1 = cube2.springs[k].m1;
            
            if ((count(map2.begin(), map2.end(), p0)) && (count(map2.begin(), map2.end(), p1))){
                int itr = find(map2_springs.begin(), map2_springs.end(), k)-map2_springs.begin();
                int itr2 = find(map2.begin(), map2.end(), p0)-map2.begin();
                int itr3 = find(map2.begin(), map2.end(), p1)-map2.begin();
                cube2.springs[k].m0 = cube1.masses[map1[itr2]].ID;
                cube2.springs[k].m1 = cube1.masses[map1[itr3]].ID;
                cube2.springs[k].ID = cube1.springs[map1_springs[itr]].ID;
                cube2.springIDs.push_back(cube1.springs[map1_springs[itr]].ID);
            }
            else if (find(map2.begin(), map2.end(), p0) != map2.end()){
                int itr = find(map2.begin(), map2.end(), p0)-map2.begin();
                cube2.springs[k].m0 = cube1.masses[map1[itr]].ID;
                cube2.springs[k].m1 = cube2.masses[p1].ID;
                cube2.springs[k].ID = springs.size();
                cube2.springIDs.push_back(springs.size());
                springs.push_back(cube2.springs[k]);
            }
            else if (find(map2.begin(), map2.end(), p1) != map2.end()){
                int itr = find(map2.begin(), map2.end(), p1)-map2.begin();
                cube2.springs[k].m0 = cube2.masses[p0].ID;
                cube2.springs[k].m1 = cube1.masses[map1[itr]].ID;
                cube2.springs[k].ID = springs.size();
                cube2.springIDs.push_back(springs.size());
                springs.push_back(cube2.springs[k]);
            }
            else{
                cube2.springs[k].m0 = cube2.masses[p0].ID;
                cube2.springs[k].m1 = cube2.masses[p1].ID;
                cube2.springs[k].ID = springs.size();
                cube2.springIDs.push_back(springs.size());
                springs.push_back(cube2.springs[k]);
            }
        }
    }
    
}

void initialize_cube(Cube &cube){
    vector<PointMass> masses;
    vector<Spring> springs;
    
    initialize_masses(masses);
    initialize_springs(springs);
    
    cube.masses = masses;
    cube.springs = springs;
    
}

void initialize_masses(vector<PointMass> &masses){
    //Point Mass of bottom, front left vertex
    //----------------------
    PointMass mass0;
    mass0.mass = 1.0f;
    mass0.position = {-0.25f, -0.25f, 0.0f};
    mass0.velocity = {0.0f, 0.0f, 0.0f};
    mass0.acceleration = {0.0f, 0.0f, 0.0f};
    mass0.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, back left vertex
    //----------------------
    PointMass mass1;
    mass1.mass = 1.0f;
    mass1.position = {-0.25f, 0.25f, 0.0f};
    mass1.velocity = {0, 0, 0};
    mass1.acceleration = {0, 0, 0};
    mass1.forces = {0, 0, 0};
    //----------------------
    
    //Point Mass of bottom, back right vertex
    //----------------------
    PointMass mass2;
    mass2.mass = 1.0f;
    mass2.position = {0.25f, 0.25f, 0.0f};
    mass2.velocity = {0.0f, 0.0f, 0.0f};
    mass2.acceleration = {0.0f, 0.0f, 0.0f};
    mass2.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, front right vertex
    //----------------------
    PointMass mass3;
    mass3.mass = 1.0f;
    mass3.position = {0.25f, -0.25f, 0.0f};
    mass3.velocity = {0.0f, 0.0f, 0.0f};
    mass3.acceleration = {0.0f, 0.0f, 0.0f};
    mass3.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, front left vertex
    //----------------------
    PointMass mass4;
    mass4.mass = 1.0f;
    mass4.position = {-0.25f, -0.25f, 0.5f};
    mass4.velocity = {0.0f, 0.0f, 0.0f};
    mass4.acceleration = {0.0f, 0.0f, 0.0f};
    mass4.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, back left vertex
    //----------------------
    PointMass mass5;
    mass5.mass = 1.0f;
    mass5.position = {-0.25f, 0.25f, 0.5f};
    mass5.velocity = {0.0f, 0.0f, 0.0f};
    mass5.acceleration = {0.0f, 0.0f, 0.0f};
    mass5.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, back right vertex
    //----------------------
    PointMass mass6;
    mass6.mass = 1.0f;
    mass6.position = {0.25f, 0.25f, 0.5f};
    mass6.velocity = {0.0f, 0.0f, 0.0f};
    mass6.acceleration = {0.0f, 0.0f, 0.0f};
    mass6.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, front right vertex
    //----------------------
    PointMass mass7;
    mass7.mass = 1.0f;
    mass7.position = {0.25f, -0.25f, 0.5f};
    mass7.velocity = {0.0f, 0.0f, 0.0f};
    mass7.acceleration = {0.0f, 0.0f, 0.0f};
    mass7.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    masses = {mass0, mass1, mass2, mass3, mass4, mass5, mass6, mass7};
    
}

void initialize_springs(vector<Spring> &springs){
    
    //Bottom Face of the Cube
    //-----------------------
    Spring spring0;
    spring0.L0 = 0.5f;
    spring0.L = 0.5f;
    spring0.k = spring_constant;
    spring0.m0 = 0;
    spring0.m1 = 1;
    spring0.original_L0 = 0.5f;
    
    Spring spring1;
    spring1.L0 = 0.5f;
    spring1.L = 0.5f;
    spring1.k = spring_constant;
    spring1.m0 = 1;
    spring1.m1 = 2;
    spring1.original_L0 = 0.5f;
    
    Spring spring2;
    spring2.L0 = 0.5f;
    spring2.L = 0.5f;
    spring2.k = spring_constant;
    spring2.m0 = 2;
    spring2.m1 = 3;
    spring2.original_L0 = 0.5f;
    
    Spring spring3;
    spring3.L0 = 0.5f;
    spring3.L = 0.5f;
    spring3.k = spring_constant;
    spring3.m0 = 3;
    spring3.m1 = 0;
    spring3.original_L0 = 0.5f;
    //----------------------
    
    //Cross Springs of Bottom Face
    //----------------------
    Spring spring4;
    spring4.L0 = 0.5f*sqrt(2.0f);
    spring4.L = 0.5f*sqrt(2.0f);
    spring4.k = spring_constant;
    spring4.m0 = 0;
    spring4.m1 = 2;
    spring4.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring5;
    spring5.L0 = 0.5f*sqrt(2.0f);
    spring5.L = 0.5f*sqrt(2.0f);
    spring5.k = spring_constant;
    spring5.m0 = 1;
    spring5.m1 = 3;
    spring5.original_L0 = 0.5f*sqrt(2.0f);
    //----------------------
    
    //Vertical Supports of Cube
    //----------------------
    Spring spring6;
    spring6.L0 = 0.5f;
    spring6.L = 0.5f;
    spring6.k = spring_constant;
    spring6.m0 = 0;
    spring6.m1 = 4;
    spring6.original_L0 = 0.5f;
    
    Spring spring7;
    spring7.L0 = 0.5f;
    spring7.L = 0.5f;
    spring7.k = spring_constant;
    spring7.m0 = 1;
    spring7.m1 = 5;
    spring7.original_L0 = 0.5f;
    
    Spring spring8;
    spring8.L0 = 0.5f;
    spring8.L = 0.5f;
    spring8.k = spring_constant;
    spring8.m0 = 2;
    spring8.m1 = 6;
    spring8.original_L0 = 0.5f;
    
    Spring spring9;
    spring9.L0 = 0.5f;
    spring9.L = 0.5f;
    spring9.k = spring_constant;
    spring9.m0 = 3;
    spring9.m1 = 7;
    spring9.original_L0 = 0.5f;
    //---------------------
    
    //Cross Springs of Front Face
    //---------------------
    Spring spring10;
    spring10.L0 = 0.5f*sqrt(2.0f);
    spring10.L = 0.5f*sqrt(2.0f);
    spring10.k = spring_constant;
    spring10.m0 = 0;
    spring10.m1 = 7;
    spring10.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring11;
    spring11.L0 = 0.5f*sqrt(2.0f);
    spring11.L = 0.5f*sqrt(2.0f);
    spring11.k = spring_constant;
    spring11.m0 = 3;
    spring11.m1 = 4;
    spring11.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Left Face
    //---------------------
    Spring spring12;
    spring12.L0 = 0.5f*sqrt(2.0f);
    spring12.L = 0.5f*sqrt(2.0f);
    spring12.k = spring_constant;
    spring12.m0 = 0;
    spring12.m1 = 5;
    spring12.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring13;
    spring13.L0 = 0.5f*sqrt(2.0f);
    spring13.L = 0.5f*sqrt(2.0f);
    spring13.k = spring_constant;
    spring13.m0 = 1;
    spring13.m1 = 4;
    spring13.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Back Face
    //---------------------
    Spring spring14;
    spring14.L0 = 0.5f*sqrt(2.0f);
    spring14.L = 0.5f*sqrt(2.0f);
    spring14.k = spring_constant;
    spring14.m0 = 1;
    spring14.m1 = 6;
    spring14.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring15;
    spring15.L0 = 0.5f*sqrt(2.0f);
    spring15.L = 0.5f*sqrt(2.0f);
    spring15.k = spring_constant;
    spring15.m0 = 2;
    spring15.m1 = 5;
    spring15.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Right Face
    //---------------------
    Spring spring16;
    spring16.L0 = 0.5f*sqrt(2.0f);
    spring16.L = 0.5f*sqrt(2.0f);
    spring16.k = spring_constant;
    spring16.m0 = 2;
    spring16.m1 = 7;
    spring16.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring17;
    spring17.L0 = 0.5f*sqrt(2.0f);
    spring17.L = 0.5f*sqrt(2.0f);
    spring17.k = spring_constant;
    spring17.m0 = 3;
    spring17.m1 = 6;
    spring17.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Top Face of the Cube
    //---------------------
    Spring spring18;
    spring18.L0 = 0.5f;
    spring18.L = 0.5f;
    spring18.k = spring_constant;
    spring18.m0 = 4;
    spring18.m1 = 5;
    spring18.original_L0 = 0.5f;
    
    Spring spring19;
    spring19.L0 = 0.5f;
    spring19.L = 0.5f;
    spring19.k = spring_constant;
    spring19.m0 = 5;
    spring19.m1 = 6;
    spring19.original_L0 = 0.5f;
    
    Spring spring20;
    spring20.L0 = 0.5f;
    spring20.L = 0.5f;
    spring20.k = spring_constant;
    spring20.m0 = 6;
    spring20.m1 = 7;
    spring20.original_L0 = 0.5f;
    
    Spring spring21;
    spring21.L0 = 0.5f;
    spring21.L = 0.5f;
    spring21.k = spring_constant;
    spring21.m0 = 7;
    spring21.m1 = 4;
    spring21.original_L0 = 0.5f;
    //---------------------
    
    //Cross Springs of Top Face
    //---------------------
    Spring spring22;
    spring22.L0 = 0.5f*sqrt(2.0f);
    spring22.L = 0.5f*sqrt(2.0f);
    spring22.k = spring_constant;
    spring22.m0 = 4;
    spring22.m1 = 6;
    spring22.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring23;
    spring23.L0 = 0.5f*sqrt(2.0f);
    spring23.L = 0.5f*sqrt(2.0f);
    spring23.k = spring_constant;
    spring23.m0 = 5;
    spring23.m1 = 7;
    spring23.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Inner Cross Springs
    //---------------------
    Spring spring24;
    spring24.L0 = 0.5f*sqrt(3.0f);
    spring24.L = 0.5f*sqrt(3.0f);
    spring24.k = spring_constant;
    spring24.m0 = 0;
    spring24.m1 = 6;
    spring24.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring25;
    spring25.L0 = 0.5f*sqrt(3.0f);
    spring25.L = 0.5f*sqrt(3.0f);
    spring25.k = spring_constant;
    spring25.m0 = 2;
    spring25.m1 = 4;
    spring25.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring26;
    spring26.L0 = 0.5f*sqrt(3.0f);
    spring26.L = 0.5f*sqrt(3.0f);
    spring26.k = spring_constant;
    spring26.m0 = 1;
    spring26.m1 = 7;
    spring26.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring27;
    spring27.L0 = 0.5f*sqrt(3.0f);
    spring27.L = 0.5f*sqrt(3.0f);
    spring27.k = spring_constant;
    spring27.m0 = 3;
    spring27.m1 = 5;
    spring27.original_L0 = 0.5f*sqrt(3.0f);
    //---------------------
    
    springs = {spring0, spring1, spring2, spring3, spring4, spring5, spring6, spring7, spring8, spring9, spring10, spring11, spring12, spring13, spring14, spring15, spring16, spring17, spring18, spring19, spring20, spring21, spring22, spring23, spring24, spring25, spring26, spring27};
}

bool compareByFitness(const Controller &control1, const Controller &control2){
    return control1.fitness > control2.fitness;
}

