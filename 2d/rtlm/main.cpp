#include "common.h"
#include "rubik13.h"
#include "rubik12.h"
#include "rubik_obs.h"
#include "json.hpp"
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
// #include "rubikILP.h"


void main_program(int argc, char* argv[]){
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputTxt;
    std::string outputTxt;
    std::string rthSolver;
    std::string use_amapf;
    std::string save_paths;
    std::string having_obstacles;

    desc.add_options()("help", "produce help message")(
    "input,i", po::value<std::string>(&inputTxt)->required(),
    "input file (txt)")("output,o",
                        po::value<std::string>(&outputTxt)->required(),
                        "output file (txt)")(
    "rth,r", po::value<std::string>(&rthSolver)->default_value("RTH_LBA"),
    "which RTH solver?")
    (
    "save_paths,s", po::value<std::string>(&save_paths)->default_value("false"),
    "save_paths?")
      (
    "obstacles,b", po::value<std::string>(&having_obstacles)->default_value("false"),
    "obstacles?")
    (
    "use_amapf,u", po::value<std::string>(&use_amapf)->default_value("false"),
    "use_amapf?");
    try{
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u) {
        std::cout << desc << "\n";
        return ;
        }
    }
    catch(po::error& e){
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return;
        // return 1;
    }

    std::unordered_set<Location> obstacles;
    Agents agents;
    int dim;
    agents_from_txt(inputTxt,agents,dim);
    bool amapf=(use_amapf=="true");
    bool saving_path=(save_paths=="true");
    // if(dim<=210) saving_path=true;
    // if(dim>210) exit(0);
    //dim=30;
    // std::cout<<"dim="<<dim<<std::endl;
    // std::cout<<"agent size="<<agents.size()<<std::endl;
    if(rthSolver=="RTH"){
        RUBIK13 rt(agents,{dim,dim});
       
        rt.set_umapf(amapf);
        rt.savePaths(saving_path);
        rt.useLba(false);
        rt.solve();
        std::cout<<"outputfile="<<outputTxt<<std::endl;
        rt.save_results(outputTxt);
    }
    else if(rthSolver=="RTH_LBA"){
        RUBIK13 rt(agents,{dim,dim});
        rt.set_umapf(amapf);
        rt.savePaths(saving_path);
        rt.solve();
        std::cout<<"outputfile="<<outputTxt<<std::endl;
        rt.save_results(outputTxt);
    }
    else if(rthSolver=="RTH_LBA_H1"){
        IRUBIK13_H1 rt(agents,{dim,dim});
        rt.savePaths(saving_path);
        rt.set_umapf(amapf);
        rt.solve();
        rt.save_results(outputTxt);
    }
    else if(rthSolver=="RTH_LBA_H2"){
        IRUBIK13_H2 rt(agents,{dim,dim});
        rt.savePaths(saving_path);
        rt.set_umapf(amapf);
        rt.solve();
        rt.save_results(outputTxt);
    }
    else if(rthSolver=="RTH_OBS"){
        RTH_obs rt(agents,{dim,dim});
        rt.useLba(false);
        rt.savePaths(saving_path);
        rt.solve();
        rt.save_results(outputTxt);

    }
    else if(rthSolver=="RTH_merge"){    //half_filling cases
        // RUBIK12 rt(agents,{dim,dim*3/2});
        RUBIK12 rt(agents,{dim,dim});
        rt.savePaths(saving_path);
        rt.set_umapf(amapf);
        rt.solve();
        
        rt.save_results(outputTxt);
    }
    else if(rthSolver=="RTH_ILP"){
        // RTH_ILP rt(agents,{dim,dim});
        // rt.savePaths(saving_path);
        // rt.set_umapf(amapf);
        // rt.solve();
        // rt.save_results(outputTxt);
    }

}




int main(int argc, char* argv[]){




    main_program(argc,argv);

    return 0;
}