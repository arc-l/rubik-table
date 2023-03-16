#ifndef RUBIK_H
#define RUBIK_H

#include"common.h"


class RUBIK{
public:
    RUBIK(){};
    ~RUBIK(){};
    RUBIK(Agents &agents, point2d dims){
        xmax=dims.first;
        ymax=dims.second;
        this->agents=agents;
    }
    RUBIK(const Configs &starts, const Configs &goals,int xmax,int ymax){
        this->starts=starts;
        this->goals=goals;
        this->xmax=xmax;
        this->ymax=ymax;
    }
    virtual void solve()=0;
    void save_results(std::string file_name){
        Paths paths;
        for(auto &agent :agents){
            paths.push_back(agent->path);
        }
        // std::cout<<"save_paths="<<save_paths<<std::endl;
        // save_result_as_txt(file_name,agents,runtime,save_paths);
        save_paths_as_json(file_name,paths,runtime);
        std::cout<<"saved successfully"<<std::endl;
    }
    void savePaths(bool save_paths){
        // std::cout<<save_paths<<std::endl;
        this->save_paths=save_paths;
    }


protected:
    int xmax;
    int ymax;
    Agents agents;
    Configs starts;
    Configs goals;
    bool save_paths=false;
    double runtime=0;

};




#endif