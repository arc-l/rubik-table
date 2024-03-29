// Linear bottleneck assignment
#ifndef LBAP_H
#define LBAP_H

#include<algorithm>
#include<iostream>
#include<vector>




double labp_solve(std::vector <std::vector<double> >& costMatrix, std::vector<int>& Assignment);

double lba_sparse(std::vector<std::tuple<int,int,double>>&costEdges, std::vector<int>& Assignment); //for solving sparse matrix

double hungarian_s(std::vector<std::tuple<int,int,double>>&costEdges, std::vector<int>& Assignment); //for solving sparse matrix

bool check_feasible_sparse(std::vector<std::tuple<int,int,double>>&costEdges,std::vector<int>&assignment,int m,double threshold);
// double labp_thresh(vector <vector<double> >& costMatrix, vector<int>& Assignment);

#endif