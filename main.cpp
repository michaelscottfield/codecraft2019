#include <iostream>
#include <vector>
#include <set>
#include <map>
#include "sstream"
#include "fstream"
#include "queue"
#include "algorithm"

using namespace std;
int t = 0;
double max_double = 9223372036854775807;

class Car {
public:
    int car_id = 0;
    int cross_from = 0;
    int cross_to = 0;
    int speed = 0;
    int plan_time = 0;
    int start_time = 0;//real leaving garage time
    int end_time = 0;
    int index_road = 0;
    int channel = 0;//所在车道 (1, 车道数)
    int car_index = 0;//在当前道路上已行驶的距离 位置(1, 道路长度)
    int priority = 0;
    int preset = 0;
    int state = 0;//车辆状态，0：等待车辆；1：终止车辆；
    vector<int> path_road; //规划的路径存放路口
    vector<int> path_cross; //规划的路径存放路口
    Car() = default;

    explicit Car(vector<int> intArray){
        car_id = intArray[0];
        cross_from = intArray[1];
        cross_to = intArray[2];
        speed = intArray[3];
        plan_time = intArray[4];
        priority = intArray[5];
        preset = intArray[6];
    }
    int get_current_road() {
        int road_id = path_road[index_road];
        return road_id;
    }

    int get_next_road(){
        int road_id = path_road[index_road + 1];
        return road_id;
    }

};

bool selfDefinedSort6( const Car *car1, const Car *car2){
    if(car1->priority > car2->priority){
        return true;
    }else if(car1->priority < car2->priority){
        return false;
    }else{
        if(car1->start_time < car2->start_time){
            return true;
        }else if(car1->start_time > car2->start_time){
            return false;
        }else{
            return car1->car_id < car2->car_id;
        }
    }
}

bool selfDefinedSort7( const Car *car1, const Car *car2){
    if(car1->priority > car2->priority){
        return true;
    }else if(car1->priority < car2->priority){
        return false;
    }else{
        if(car1->car_index > car2->car_index){
            return true;
        }else if(car1->car_index < car2->car_index){
            return false;
        }else{
            return car1->channel < car2->channel;
        }
    }
}
class Road{
public:
    int road_id = -1;
    int length = -1;
    int speed = -1;
    int channel = -1;
    int cross_from = -1;
    int cross_to = -1;
    bool isDuplex = -1;
    int num_car = 0;
    map<int, int> roads;
    map<int, int> roads_to;
    vector<Car*> car_waiting;
    vector<Car*> car_queue;
    vector<vector<int>> cars;
    Road() = default;
    explicit Road(vector<int> &intArray) {
        road_id = intArray[0];
        length = intArray[1];
        speed = intArray[2];
        channel = intArray[3];
        cross_from = intArray[4];
        cross_to = intArray[5];
        isDuplex = (intArray[6] == 1);
        cars.resize(channel + 1);
        for (int i = 1; i < channel + 1; ++i) {
            cars[i].resize(length + 1);
        }
        for (int i = 1; i < channel + 1; ++i) {
            for (int j = 1; j < length + 1; ++j) {
                cars[i][j] = 0;
                cars[i][j] = 0;
            }
        }
    }
    void init_car_waiting(){
        sort(car_waiting.begin(), car_waiting.end(), selfDefinedSort6);
    }

    void drive_car_in_current_road(Car &car, int new_car_index){
        cars[car.channel][car.car_index] = 0;
        car.car_index = new_car_index;
        cars[car.channel][car.car_index] = car.car_id;
        car.state = 1;
    }

    void drive_start_car(Car *car, int new_channel, int new_car_index, vector<int> &car_running, vector<int> &global_car_waiting){
        car->channel = new_channel;
        car->car_index = new_car_index;
        car->index_road = 0;
        car->state = 1;
        cars[car->channel][car->car_index] = car->car_id;
        num_car += 1;

        if(car->preset == 0){
            car->start_time = t;
        }
        for(auto iter = global_car_waiting.begin(); iter != global_car_waiting.end(); iter++){
            if(*iter == car->car_id) {
                global_car_waiting.erase(iter);
                break;
            }
        }
        car_running.push_back(car->car_id);
    }

    bool try_start_car(Car* car, map<int, Car> &mapCar, vector<int> &car_running, vector<int> &global_car_waiting){
        int s = car->speed < speed ? car->speed : speed;
        for(int channel_i = 1; channel_i < channel + 1; channel_i++){
            int tag_wait = 0;
            //没有空位，寻找下一车道
            if(cars[channel_i][1] != 0) continue;
            for(int car_index = 1; car_index < s + 1; car_index++){
                int tmp_car_id = cars[channel_i][car_index];
                if(tmp_car_id == 0) continue;
                Car &tmp_car = mapCar[tmp_car_id];
                if(tmp_car.state == 0){
                    tag_wait = 1;
                    break;
                }
                drive_start_car(car, channel_i, car_index - 1, car_running, global_car_waiting);
                return true;
            }
            if(tag_wait == 0){
                drive_start_car(car, channel_i, s, car_running, global_car_waiting);
                return true;
            }
        }
        return false;
    }

    void run_car_in_init_list(int priority, map<int, Car> &mapCar, vector<int> &car_running, vector<int> &global_car_waiting){
        if(priority == 1){
            int i = 0;
            while(i < car_waiting.size()){
                Car *car = car_waiting[i];
                if(car->start_time > t){
                    break;
                }
                if(car_running.size() >= 5000 && car->preset == 0){
                    i += 1;
                    continue;
                }
                if(get_density() > 0.4 && car->preset == 0){
                    i += 1;
                    continue;
                }
                if(!try_start_car(car, mapCar, car_running, global_car_waiting)){
                    break;
                }
                car_waiting.erase(car_waiting.begin() + i);
            }
        } else{
            int i = 0;
            while(i < car_waiting.size()){
                Car *car = car_waiting[i];
                if(car->start_time > t && car->priority == 0){
                    break;
                }
                if(car->start_time > t && car->priority == 1){
                    i += 1;
                    continue;
                }
                if(car_running.size() >= 4000 && car->preset == 0){
                    i += 1;
                    continue;
                }
                if(get_density() > 0.4 && car->preset == 0){
                    i += 1;
                    continue;
                }
                if(!try_start_car(car, mapCar, car_running, global_car_waiting)){
                    break;
                }
                car_waiting.erase(car_waiting.begin() + i);
            }
        }
    }

    void create_queue(map<int, Car> &mapCar){
        car_queue.clear();
        map<int ,vector<Car*>> channel_wait_car;
        for(int channel_i = 1; channel_i < channel + 1; channel_i++){
            for(int car_index = length; car_index > 0; car_index--){
                int tmp_car_id = cars[channel_i][car_index];
                if(tmp_car_id == 0) continue;
                Car* tmp_car = &mapCar[tmp_car_id];
                if(tmp_car->car_index + (tmp_car->speed < speed ? tmp_car->speed : speed) <= length){
                    break;
                }
                if(tmp_car->state == 1) break;
                channel_wait_car[channel_i].push_back(tmp_car);
            }
        }
        vector<Car*> car_list;
        for(auto iter : channel_wait_car){
            if(iter.second.empty()) continue;
            car_list.push_back(*(iter.second.begin()));
            iter.second.erase(iter.second.begin());
        }
        while(!car_list.empty()){
            sort(car_list.begin(), car_list.end(), selfDefinedSort7);
            Car *first_car = *car_list.begin();
            car_list.erase(car_list.begin());
            car_queue.push_back(first_car);
            if(channel_wait_car[first_car->channel].empty()) continue;
            car_list.push_back(*channel_wait_car[first_car->channel].begin());
            channel_wait_car[first_car->channel].erase(channel_wait_car[first_car->channel].begin());
        }
    }

    int get_car_from_queue(){
        if(car_queue.empty()){
            return -1;
        }
        return car_queue[0]->car_id;
    }

    double get_density(){
        return (double)num_car / ((double) channel * (double)length);
    }
};

class Cross{
public:
    int cross_id = -1;
    vector<int> roads_id;
    map<int, int> cross_road;
    //进路口道路号和roads_id顺序一样
    vector<int> roads_in_id;
    Cross() = default;

    explicit Cross(vector<int> &intArray, map<int, Road> &mapRoad) {
        cross_id = intArray[0];
        roads_id.push_back(intArray[1]);
        roads_id.push_back(intArray[2]);
        roads_id.push_back(intArray[3]);
        roads_id.push_back(intArray[4]);
        set_cross_road(mapRoad);
        set_roads_in(mapRoad);
    }

    void set_cross_road(map<int, Road> &mapRoad){
        for(auto road_id : roads_id){
            if(road_id == -1) continue;
            Road &road = mapRoad[road_id];
            if(cross_id == road.cross_to && road.isDuplex == 0){
                continue;
            }
            int road_adjacent = road.roads[cross_id];
            int cross_adjacent = road.cross_to;
            if(cross_id == road.cross_to){
                cross_adjacent = road.cross_from;
            }
            cross_road[cross_adjacent] = road_adjacent;
        }
    }

    void set_roads_in(map<int, Road> &mapRoad){
        for(auto road_id : roads_id){
            if(road_id == -1){
                roads_in_id.push_back(-1);
                continue;
            }
            Road &road = mapRoad[road_id];
            if(road.roads_to.find(cross_id) != road.roads_to.end()){
                roads_in_id.push_back(road.roads_to[cross_id]);
            }else{
                roads_in_id.push_back(-1);
            }
        }
    }

    vector<int> get_roads_in(map<int, Road> &mapRoad){
        vector<int> result;
        vector<int> new_roads_id;
        for(auto road_id : roads_id){
            if(road_id == -1) continue;
            new_roads_id.push_back(road_id);
        }
        sort(new_roads_id.begin(), new_roads_id.end());
        for(auto road_id : new_roads_id){
            Road &road = mapRoad[road_id];
            if(road.roads_to.find(cross_id) != road.roads_to.end()){
                result.push_back(road.roads_to[cross_id]);
            }else{
                continue;
            }
        }
        return result;
    }

    int get_index(int road_id){
        for(int i = 0; i < roads_id.size(); i++){
            if(roads_id[i] == road_id){
                return i;
            }
        }
        return -1;
    }

    int get_out_direction(Car &car){
        if(car.index_road == car.path_road.size() - 1){
            return 2;
        }
        int road_from = car.path_road[car.index_road];
        if(road_from > 10000){
            road_from -= 10000;
        }
        int road_to = car.path_road[car.index_road + 1];
        if(road_to > 10000){
            road_to -= 10000;
        }
        int index_from = get_index(road_from);
        int index_to = get_index(road_to);
        int tmp = index_to - index_from;
        if(tmp == 2 || tmp == -2) return 2;
        if(tmp == -1 || tmp == 3) return 4;
        if(tmp == 1 || tmp == -3) return 3;
        return -1;
    }

    int get_right(int road_id){
        if(road_id > 10000) road_id -= 10000;
        int road_index = get_index(road_id);
        int next_road_index = road_index -1;
        if(next_road_index == -1){
            next_road_index = 3;
        }
        if(roads_in_id[next_road_index] == -1) {
            return -1;
        }else{
            return roads_in_id[next_road_index];
        }
    }

    int get_left(int road_id){
        if(road_id > 10000) road_id -= 10000;
        int road_index = get_index(road_id);
        int next_road_index = road_index + 1;
        if(next_road_index == 4){
            next_road_index = 0;
        }
        if(roads_in_id[next_road_index] == -1) {
            return -1;
        }else{
            return roads_in_id[next_road_index];
        }
    }

    int get_front(int road_id){
        if(road_id > 10000){
            road_id -= 10000;
        }
        int road_index = get_index(road_id);
        int next_road_index = road_index + 2;
        if(road_index >= 2){
            next_road_index = road_index - 2;
        }
        if(roads_in_id[next_road_index] == -1) {
            return -1;
        }else{
            return roads_in_id[next_road_index];
        }
    }

    bool conflict(Road &road, Car &car, map<int, Road> &mapRoad, map<int, Car> &mapCar){
        int out_direction = get_out_direction(car);
        if(out_direction == 3){
            int temp_road_id = get_right(road.road_id);
            if(temp_road_id != -1){
                Road &right_road = mapRoad[temp_road_id];
                int temp_car_id = right_road.get_car_from_queue();
                if(temp_car_id != -1){
                    Car &first_car = mapCar[temp_car_id];
                    int first_out_direction = get_out_direction(first_car);
                    if(car.priority == 1){
                        if(first_car.priority == 1 && first_out_direction == 2){
                            return true;
                        }
                    } else{
                        if(first_out_direction == 2){
                            return true;
                        }
                    }
                }
            }
        } else if(out_direction == 4){
            int temp_road_id = get_left(road.road_id);
            if(temp_road_id != -1){
                Road &left_road = mapRoad[temp_road_id];
                int temp_car_id = left_road.get_car_from_queue();
                if(temp_car_id != -1){
                    Car &first_car = mapCar[temp_car_id];
                    int first_out_direction = get_out_direction(first_car);
                    if(car.priority == 1){
                        if(first_car.priority == 1 && first_out_direction == 2){
                            return true;
                        }
                    } else{
                        if(first_out_direction == 2){
                            return true;
                        }
                    }
                }
            }
            temp_road_id = get_front(road.road_id);
            if(temp_road_id != -1){
                Road &front_road = mapRoad[temp_road_id];
                int temp_car_id = front_road.get_car_from_queue();
                if(temp_car_id != -1){
                    Car &first_car = mapCar[temp_car_id];
                    int first_out_direction = get_out_direction(first_car);
                    if(car.priority == 1){
                        if(first_car.priority == 1 && first_out_direction == 3){
                            return true;
                        }
                    } else{
                        if(first_out_direction == 3){
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }
};

void dijkstra(Car &car, int car_from, int car_to, vector<int> &roads_id, map<int, Cross> &mapCross, map<int, Road> &mapRoad){
    map<int, double> distance;
    map<int, bool> visit;
    map<int, vector<int>> path;
    vector<int> temp_vec = {car_from};
    for(auto &iter : mapCross){
        distance.insert(make_pair(iter.first, max_double));
        visit.insert(make_pair(iter.first, false));
        path.insert(make_pair(iter.first, temp_vec));
    }
    distance[car_from] = 0.0;
    set<pair<double, int>> q;
    q.insert(make_pair(distance[car_from], car_from));
    while (!q.empty()){
        int cross_id;
        //double dis = q.begin()->first;
        cross_id = q.begin()->second;
        q.erase(q.begin());
        if(visit[cross_id]) continue;
        visit[cross_id] = true;
        vector<int> path_cross;
        path_cross.assign(path[cross_id].begin(), path[cross_id].end());
        Cross &cross = mapCross[cross_id];
        for(auto tmp_cross_road : cross.cross_road){
            int cross_adjacent_id = tmp_cross_road.first;
            int road_id = tmp_cross_road.second;
            double weight;
            if(find(roads_id.begin(), roads_id.end(), road_id) != roads_id.end()){
                weight = 1000.0;
            } else{
                Road &road = mapRoad[road_id];
                double density = road.get_density();
                weight = density * 0.99 + ((double)road.length / (double)car.speed) * 0.01;
            }
            double dis_adjacent = distance[cross_id] + weight;
            if(dis_adjacent < distance[cross_adjacent_id] && !visit[cross_adjacent_id]){
                distance[cross_adjacent_id] = dis_adjacent;
                q.insert(make_pair(distance[cross_adjacent_id], cross_adjacent_id));
                vector<int> path_tmp;
                path_tmp.assign(path_cross.begin(), path_cross.end());
                path_tmp.push_back(cross_adjacent_id);
                path[cross_adjacent_id] = path_tmp;
            }
        }
    }
    car.path_cross = path[car_to];
    car.path_road.clear();
    for(int i = 0; i < car.path_cross.size() - 1; i++){
        int current_cross_id = car.path_cross[i];
        Cross &current_cross = mapCross[current_cross_id];
        int next_cross_id = car.path_cross[i + 1];
        int road_id = current_cross.cross_road[next_cross_id];
        car.path_road.push_back(road_id);
    }
}

bool drive_set_car(Road & road, Car &car, map<int, Car> &mapCar, map<int, Cross> &mapCross, map<int, Road> &mapRoad){
    bool con = true;//true for in
    int s = car.speed < road.speed ? car.speed : road.speed;
    if(car.car_index + s > road.length){
        con = false;
        s = road.length - car.car_index;
    }
    int start = car.car_index + 1;
    int end = start + s;
    for(auto car_index = start; car_index < end; car_index++){
        int tmp_car_id = road.cars[car.channel][car_index];
        if(tmp_car_id == 0) continue;
        Car &tmp_car = mapCar[tmp_car_id];
        if(tmp_car.state == 0){
            car.state = 0;
            return false;//waiting
        } else{
            road.drive_car_in_current_road(car, car_index - 1);
            return true;//terminated
        }
    }
    if(con){
        road.drive_car_in_current_road(car, car.car_index + s);
        return true;
    } else{
        if(car.preset == 1 || car.path_cross[car.index_road + 1] == car.cross_to){
            car.state = 0;
            return false;
        }
        //在第一个路口时重新规划路线
        //if(car.index_road % 2 == 0){
        if(car.index_road == 0 || mapRoad[car.path_road[car.index_road + 1]].get_density() > 0.4){
            int road_id = car.path_road[car.index_road];
            if(road_id > 10000){
                road_id = road_id - 10000;
            } else{
                road_id = road_id + 10000;
            }
            vector<int> temp_vec = {road_id};
            vector<int> temp_road;
            temp_road.assign(car.path_road.begin(), car.path_road.begin() + car.index_road + 1);
            vector<int> temp_cross;
            temp_cross.assign(car.path_cross.begin(), car.path_cross.begin() + car.index_road + 1);
            dijkstra(car, car.path_cross[car.index_road + 1], car.cross_to, temp_vec, mapCross, mapRoad);
            car.path_road.insert(car.path_road.begin(), temp_road.begin(), temp_road.end());
            car.path_cross.insert(car.path_cross.begin(), temp_cross.begin(), temp_cross.end());
        }
        car.state = 0;
        return false;
    }
}

void drive_set_car_by_channel(Road &road, int new_channel, map<int, Car> &mapCar, map<int, Cross> &mapCross, map<int, Road> &mapRoad){
    for(int car_index = road.length; car_index > 0; car_index--){
        int car_id = road.cars[new_channel][car_index];
        if(car_id == 0) continue;
        Car &car = mapCar[car_id];
        drive_set_car(road, car, mapCar, mapCross, mapRoad);
    }
}

void drive_car_by_channel(Road &road, int new_channel, map<int, Car> &mapCar, map<int, Cross> &mapCross, map<int, Road> &mapRoad){
    for(int car_index = road.length; car_index > 0; car_index--){
        int car_id = road.cars[new_channel][car_index];
        if(car_id == 0) continue;
        Car &car = mapCar[car_id];
        if(car.state == 1) continue;
        drive_set_car(road, car, mapCar, mapCross, mapRoad);
    }
}

void init_car_data(map<int, Car> &mapCar, map<int, Road> &mapRoad){
    for(auto &iter : mapCar){
        Car *car = &iter.second;
        if(car->path_road.empty()) continue;
        int first_road_id = car->path_road[0];
        Road &first_road = mapRoad[first_road_id];
        first_road.car_waiting.push_back(car);
    }
    for(auto &iter : mapRoad){
        iter.second.init_car_waiting();
    }
}

void road_car_sort(map<int, Road> &mapRoad){
    for(auto &iter : mapRoad){
        for(auto &car : iter.second.car_waiting){
            if(car->start_time < t && car->preset == 0){
                car->start_time = t;
            }
        }
        iter.second.init_car_waiting();
    }
}

void plan_path(Car &car, map<int, Cross> &mapCross, map<int, Road> &mapRoad){
    vector<int> temp_vec;
    dijkstra(car, car.cross_from, car.cross_to, temp_vec, mapCross, mapRoad);
}

void plan_path_all(map<int, Car> &mapCar,map<int, Cross> &mapCross, map<int, Road> &mapRoad){
    for(auto &iter : mapCar){
        if(iter.second.preset == 1) continue;
        plan_path(iter.second, mapCross, mapRoad);
        iter.second.start_time = iter.second.plan_time;
    }
}

void drive_just_current_road(map<int, Road> &mapRoad, map<int, Car> &mapCar, map<int, Cross> &mapCross){
    for(auto &iter : mapRoad){
        for(int channel = 1; channel < iter.second.channel + 1; channel++){
            drive_set_car_by_channel(iter.second, channel, mapCar, mapCross, mapRoad);
        }
    }
}

void drive_car_init_list(int priority, map<int, Road> &mapRoad, map<int, Car> &mapCar, vector<int> &car_running, vector<int> &global_car_waiting){
    for(auto &iter : mapRoad){
        iter.second.run_car_in_init_list(priority, mapCar, car_running, global_car_waiting);
    }
}

void create_car_queue(map<int, Road> &mapRoad, map<int, Car> &mapCar){
    for(auto &iter : mapRoad){
        iter.second.create_queue(mapCar);
    }
}

int compute_out(Car &car, map<int, Road> &mapRoad){
    Road &road = mapRoad[car.get_current_road()];
    Road &next_road = mapRoad[car.get_next_road()];
    int v = car.speed;
    int r1 = road.speed;
    int r2 = next_road.speed;
    int s1 = road.length - car.car_index;
    int v1 = r1 < v ? r1 : v;
    int v2 = r2 < v ? r2 : v;
    if(s1 > v1){
        return 0;
    }
    return v2 - s1;
}

void drive_car_to_next_road(Road &road, Road &next_road, Car &car, int new_channel, int new_car_index){
    road.cars[car.channel][car.car_index] = 0;
    road.num_car -= 1;
    car.channel = new_channel;
    car.car_index = new_car_index;
    car.index_road += 1;
    car.state = 1;
    next_road.cars[car.channel][car.car_index] = car.car_id;
    next_road.num_car += 1;
}

bool move_to_next_road(map<int, Road> &mapRoad, map<int, Car> &mapCar, Car &car, vector<int> &car_running, vector<int> &car_completed){
    Road &road = mapRoad[car.get_current_road()];
    if(car.index_road == car.path_road.size() - 1){
        road.cars[car.channel][car.car_index] = 0;
        road.num_car -= 1;
        car.end_time = t;
        for(auto iter = car_running.begin(); iter != car_running.end(); iter++){
            if(*iter == car.car_id){
                car_running.erase(iter);
                break;
            }
        }
        car_completed.push_back(car.car_id);
        return true;
    }
    int s2 = compute_out(car, mapRoad);
    if(s2 <= 0){
        road.drive_car_in_current_road(car, road.length);
        return true;
    }else{
        Road &next_road = mapRoad[car.get_next_road()];
        int tag_next_road_full = 0;
        int tag_next_road_wait = 0;
        for(int channel = 1; channel < next_road.channel + 1; channel++){
            for(int car_index = 1; car_index < s2 + 1; car_index++){
                int tmp_car_id = next_road.cars[channel][car_index];
                if(tmp_car_id == 0){
                    continue;
                }
                Car &tmp_car = mapCar[tmp_car_id];
                if(tmp_car.state == 1 && tmp_car.car_index > 1){
                    drive_car_to_next_road(road, next_road, car, channel, car_index - 1);
                    return true;
                }else if(tmp_car.state == 1 && tmp_car.car_index == 1){
                    tag_next_road_full += 1;
                }else if(tmp_car.state == 0){
                    tag_next_road_wait += 1;
                }
                break;
            }
            if(tag_next_road_full + tag_next_road_wait < channel){
                drive_car_to_next_road(road, next_road, car, channel, s2);
                return true;
            }
        }
        if (tag_next_road_full == next_road.channel){
            road.drive_car_in_current_road(car, road.length);
            return true;
        }
        if(tag_next_road_wait == next_road.channel) return false;
        return false;
    }
}

bool drive_car_in_wait_state(map<int, Cross> &mapCross, map<int, Road> &mapRoad,map<int, Car> &mapCar, vector<int> &car_running, vector<int> &car_completed, vector<int> &global_car_waiting){
    vector<int> cross_list;
    cross_list.resize(mapCross.size());
    int i = 0;
    for(auto &iter : mapCross){
        cross_list[i] =iter.first;
        i++;
    }
    sort(cross_list.begin(), cross_list.end());
    int num_run = 0;
    while(!cross_list.empty()){
        vector<int> cross_uncompleted;
        for(auto &cross_id:cross_list){
            int tag_wait = 0;
            Cross &cross = mapCross[cross_id];
            for(auto &road_id : cross.get_roads_in(mapRoad)){
                Road &road = mapRoad[road_id];
                while(true){
                    int tmp_car_id = road.get_car_from_queue();
                    if(tmp_car_id == -1) break;
                    Car &car = mapCar[tmp_car_id];
                    if(cross.conflict(road, car, mapRoad, mapCar)){
                        tag_wait = 1;
                        break;
                    }
                    int channel = car.channel;
                    if(move_to_next_road(mapRoad, mapCar, car, car_running, car_completed)){
                        drive_car_by_channel(road, channel, mapCar, mapCross, mapRoad);
                        road.create_queue(mapCar);
                        road.run_car_in_init_list(true, mapCar, car_running, global_car_waiting);
                    } else{
                        tag_wait = 1;
                        break;
                    }
                }
            }
            if(tag_wait == 1){
                cross_uncompleted.push_back(cross_id);
            }
        }
        cross_list.clear();
        cross_list.assign(cross_uncompleted.begin(), cross_uncompleted.end());
        num_run += 1;
        if(num_run >= 100){
            cout << "dead lock"<<endl;
            return false;
        }
    }
    return true;
}

bool is_finish(vector<int> &car_running, vector<int> &car_waiting){
    return car_running.empty() && car_waiting.empty();
}

bool run(map<int, Car> &mapCar,map<int, Cross> &mapCross, map<int, Road> &mapRoad, vector<int> &car_running, vector<int> &car_completed, vector<int> &global_car_waiting){
    plan_path_all(mapCar, mapCross, mapRoad);
    init_car_data(mapCar, mapRoad);
    while(true){
        t += 1;
        road_car_sort(mapRoad);
        drive_just_current_road(mapRoad, mapCar, mapCross);
        drive_car_init_list(true, mapRoad, mapCar, car_running, global_car_waiting);
        create_car_queue(mapRoad, mapCar);
        if(!drive_car_in_wait_state(mapCross, mapRoad, mapCar, car_running, car_completed, global_car_waiting)){
            return false;
        }
        drive_car_init_list(false, mapRoad, mapCar, car_running, global_car_waiting);
        cout << t<<": car completed: "<<car_completed.size()<<"; car running: "<<car_running.size()<<"; car waiting: "<<global_car_waiting.size()<<endl;
        if (is_finish(car_running, global_car_waiting)) break;
    }
    return true;
}

void get_t(map<int, Car> &mapCar){
    //优先车辆最早计划时间
    int t_pri_start = 100000;
    //优先车辆最晚到达时间
    int t_pri_end = 0;
    //系统调度时间
    int t_total = 0;
    //所有车辆调度总时间
    int t_sum = 0;
    //优先车辆调度总时间
    int t_sum_pri = 0;
    for(auto &iter : mapCar){
        Car &car = iter.second;
        t_sum += car.end_time - car.plan_time;
        if(car.end_time > t_total){
            t_total = car.end_time;
        }
        if(car.priority == 1){
            if(car.plan_time < t_pri_start){
                t_pri_start = car.plan_time;
            }
            if(car.end_time > t_pri_end){
                t_pri_end = car.end_time;
            }
            t_sum_pri += car.end_time - car.plan_time;
        }
    }
    int t_pri = t_pri_end - t_pri_start;
    cout << "T: "<<t_total<<"; T_sum: "<<t_sum<<endl;
    cout << "Tpri: "<<t_pri<<"; Tsumpri: "<<t_sum_pri<<endl;
}

map<int, Road> read_from_road(const string &path) {
    ifstream in(path);
    string line;
    getline(in, line);
    map<int, Road> map1;
    while (getline(in, line)) {
        line = line.substr(1, line.size() - 2);
        stringstream ss(line);
        string str;
        vector<int> intArray;
        while (getline(ss, str, ','))intArray.push_back(stoi(str));
        int id = intArray[0];
        Road road(intArray);
        road.roads[road.cross_from] = id;
        road.roads_to[road.cross_to] = id;
        road.roads[road.cross_to] = id + 10000;
        road.roads_to[road.cross_from] = id + 10000;
        map1.insert(make_pair(id, road));
        if(!road.isDuplex) continue;
        Road road2(intArray);
        road2.road_id = road2.road_id + 10000;
        road2.cross_from = intArray[5];
        road2.cross_to = intArray[4];
        map1.insert(make_pair(id + 10000, road2));
    }
    in.close();
    return map1;
}

map<int, Car> read_from_car(const string &path) {
    ifstream in(path);
    string line;
    getline(in, line);
    map<int, Car> map1;
    while (getline(in, line)) {
        line = line.substr(1, line.size() - 2);
        stringstream ss(line);
        string str;
        vector<int> intArray;
        while (getline(ss, str, ','))intArray.push_back(stoi(str));
        int id = intArray[0];
        Car car(intArray);
        map1.insert(make_pair(id, car));
    }
    in.close();
    return map1;
}

map<int, Cross> read_from_cross(const string &path, map<int, Road> &mapRoad) {
    ifstream in(path);
    string line;
    getline(in, line);
    map<int, Cross> map1;
    while (getline(in, line)) {
        line = line.substr(1, line.size() - 2);
        stringstream ss(line);
        string str;
        vector<int> intArray;
        while (getline(ss, str, ','))intArray.push_back(stoi(str));
        int id = intArray[0];
        Cross cross(intArray, mapRoad);
        map1.insert(make_pair(id, cross));
    }
    in.close();
    return map1;
}

void read_from_preset_answer(const string &path, map<int, Car> &mapCar, map<int, Road> &mapRoad) {
    ifstream in(path);
    string line;
    getline(in, line);
    while (getline(in, line)) {
        line = line.substr(1, line.size() - 2);
        stringstream ss(line);
        string str;
        vector<int> intArray;
        while (getline(ss, str, ','))intArray.push_back(stoi(str));
        int car_id = intArray[0];
        Car &car = mapCar[car_id];
        car.start_time = intArray[1];
        vector<int> path_road;
        vector<int> path_cross;

        int cross_from = car.cross_from;
        path_cross.push_back(cross_from);
        for(int i = 2; i < intArray.size(); i++){
            int road_id = intArray[i];
            Road &road = mapRoad[road_id];
            int tmp_road_id = road.roads[cross_from];
            path_road.push_back(tmp_road_id);
            Road &tmp_road = mapRoad[tmp_road_id];
            cross_from = tmp_road.cross_to;
            path_cross.push_back(cross_from);
        }
        car.path_road = path_road;
        car.path_cross = path_cross;
    }
    in.close();
}

void write_to_answer(const string &path, map<int, Car> &mapCar) {
    ofstream out(path);
    out << "#(carId,StartTime,RoadId...)" << endl;
    for (auto &iter: mapCar) {
        string str = "(" + to_string(iter.first);
        str += ", " + to_string(iter.second.start_time);
        for (int i : iter.second.path_road) {
            if(i > 10000) i -= 10000;
            str += ", " + to_string(i);
        }
        str += ")";
        out << str << endl;
    }
    out.close();
}

int main(int argc, char *argv[]) {
    std::cout << "Begin" << std::endl;
    if(argc < 5){
        std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
        exit(1);
    }

    std::string carPath(argv[1]);
    std::string roadPath(argv[2]);
    std::string crossPath(argv[3]);
    std::string presetAnswerPath(argv[4]);
    std::string answerPath(argv[5]);

    std::cout << "carPath is " << carPath << std::endl;
    std::cout << "roadPath is " << roadPath << std::endl;
    std::cout << "crossPath is " << crossPath << std::endl;
    std::cout << "presetAnswerPath is " << presetAnswerPath << std::endl;
    std::cout << "answerPath is " << answerPath << std::endl;

    map<int, Car> mapCar = read_from_car(carPath);
    map<int, Road> mapRoad = read_from_road(roadPath);
    map<int, Cross> mapCross = read_from_cross(crossPath, mapRoad);
    read_from_preset_answer(presetAnswerPath, mapCar, mapRoad);
    vector<int> car_running;
    vector<int> car_completed;
    vector<int> car_waiting;
    car_waiting.resize(mapCar.size());
    int i = 0;
    for(auto &iter : mapCar){
        car_waiting[i] = iter.first;
        i++;
    }
    run(mapCar, mapCross, mapRoad, car_running, car_completed, car_waiting);
    get_t(mapCar);
    write_to_answer(answerPath, mapCar);
    return 0;
}