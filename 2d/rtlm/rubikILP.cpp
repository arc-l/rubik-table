/**
 * @file rubikILP.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "rubikILP.h"

void RTH_ILP::matching(){
    GRBEnv *envp = &genv;
    GRBModel model=GRBModel(*envp);
    VarMap var_map;
    prepare_model(model,var_map);
    model.optimize();
    double obj_val = model.get(GRB_DoubleAttr_ObjVal);
    for(auto &agent:agents){
        for(int row=0;row<xmax/3;row++){
            // assert(var_map.find({row,agent->id})!=var_map.end());
            if(fabs(var_map[{row,agent->id}].get(GRB_DoubleAttr_X)-1)<1e-3){
                agent->intermediate=Location(3*row+1,agent->current.y);
                break;
            }
        }
    }
}


void RTH_ILP::prepare_model(GRBModel &model, VarMap &var_map){
    using VarKey=std::pair<int,int>;
    using VarKeys=std::vector<VarKey>;
    using GRBVars=std::vector<GRBVar>;
  
    model.set(GRB_IntParam_OutputFlag, 0);
    std::unordered_map<int,GRBVars> agent_row_dict;
    std::unordered_map<point2d,GRBVars,boost::hash<point2d>> row_color_dict;
    std::unordered_map<point2d,GRBVars,boost::hash<point2d>> vertex_dict;
    

    //add variables
    for(int r=0;r<xmax/3;r++){
        for(auto&agent:agents){
            VarKey var={r,agent->id};
            GRBVar x=model.addVar(0,1,0, GRB_BINARY);
            var_map[var]=x;
            row_color_dict[{r,agent->inter_goal.y}].push_back(x);
            agent_row_dict[agent->id].push_back(x);
            vertex_dict[{agent->current.y,r}].push_back(x);
        }
    }

    //add constraints
    // agent is assigned to one row only:
    for(auto &a:agents){
        GRBLinExpr expr = GRBLinExpr();
        for(auto &var:agent_row_dict[a->id]){
            expr+=var;
        }
        if(expr.size()>1)
            model.addConstr(expr, GRB_EQUAL, 1);
    }

    //each row  the colors are different
    for(int row=0;row<xmax/3;row++){
        for(int color=0;color<ymax;color++){
            GRBLinExpr expr = GRBLinExpr();
            for(auto &var:row_color_dict[{row,color}]){
                expr+=var;
            }
            if(expr.size()>1)
                model.addConstr(expr,GRB_EQUAL,1);
        }
    }

    // each vertex must be occupied by one agent:
    for(int column=0;column<ymax;column++){
        for(int row=0;row<xmax/3;row++){
            GRBLinExpr expr = GRBLinExpr();
            for(auto &var:vertex_dict[{column,row}]){
                expr+=var;
            }
            if(expr.size()>1)
                model.addConstr(expr,GRB_EQUAL,1);
        }
    }

    // objective
    std::unordered_map<point2d,double,boost::hash<point2d>> costs;

    for(int row=0;row<xmax/3;row++){
        for(auto &a:agents){
            point2d var_key={row,a->id};
            //cost=fabs(3*row+1-a->inter_goal->x);
            double cost=fabs(3*row+1-a->current.x);
            costs[var_key]=cost;
        }
    }

    GRBVar maxobj1= model.addVar(0,GRB_INFINITY,0, GRB_INTEGER);
     
    for(auto &item:var_map){
        GRBLinExpr expr = GRBLinExpr();
        expr+=item.second *costs[item.first];
        expr-=maxobj1;
        model.addConstr(expr, GRB_LESS_EQUAL, 0);
    }
    GRBLinExpr obj_expr = GRBLinExpr();
    obj_expr+=maxobj1;
    model.setObjective(obj_expr,GRB_MINIMIZE);
    model.update();
}

