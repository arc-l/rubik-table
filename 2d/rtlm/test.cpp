#include<iostream>
#include"json.hpp"
#include<fstream>
#include<regex>
#include<random>
using namespace std;
using json=nlohmann::json;

struct position{
    int x;
    int y;
    position(int x_,int y_){
        x=x_;
        y=y_;
    }
};

void test_json(){
    std::ifstream ifs("./local3x3.json");
    json j = json::parse(ifs);
    std::cout<<j["((0, 1, 2), 'x')"]<<std::endl;
    vector<vector<int>> sol=j["((0, 1, 2), 'x')"];
    for(auto &p:sol){
        for (auto &v:p){
            cout<<v<<endl;
        }
        cout<<"???"<<endl;
    }
}

void test_vector(){
    vector<position> tests;
    position p1(1,2);
    position p2(2,3);
    tests.push_back(p1);
    tests.push_back(p2);
    p1.x=4;
    cout<<tests[0].x<<" "<<tests[0].y<<endl;

}

void test_char(){
    string c1="'x'";
    cout<<c1<<endl;
    c1=c1+'y';
    cout<<c1<<endl;

}

void test_vector2(){
    std::vector<std::vector<int>> paths;
    std::vector<int> path_i;
    paths.emplace_back(path_i);
    path_i.emplace_back(1);
    path_i.emplace_back(2);
    path_i.emplace_back(3);
    for(auto v:path_i){
        std::cout<<v<<"  ";
    }
    std::cout<<"------"<<std::endl;
    for(auto v:paths[0]){
        std::cout<<v<<"  ";
    }
    std::cout<<std::endl;
}

void test_reg(){
    std::string s="(1,2)";
    std::smatch results;
    std::regex r_vertex=std::regex(R"(((\d+),(\d+)))");
    std::regex_match(s,results,r_vertex);
    std::cout<<results.size()<<std::endl;
    for(auto result:results){
        std::cout<<result<<std::endl;
    }

}

void test_sort(){
    std::vector<int> a={5,7,9,3,1,6,8};
    auto compare=[](int i,int j){
        return i>j;
    };
    std::sort(a.begin(),a.end(),compare);
    for(auto i:a){
        std::cout<<i<<" ";
    }
    std::cout<<"\n"<<a.size()<<" "<<a.end()-a.begin()<<std::endl;
}



int main()
{
    //test_char();
    //test_json();
    //  test_vector2();
    // test_reg();
    test_sort();
}

