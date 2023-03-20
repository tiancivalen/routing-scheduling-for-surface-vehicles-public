#ifndef GENETICALGORITHM_H
#define GENETICALGORITHM_H
#include <qglobal.h>
#include "individual.h"
#include "population.h"
#include <vector>

const double c_large = 86400;
extern uint g_generation;
extern double g_fitnessalias;
extern double g_mutationrate;
extern double g_crossoverrate;

void Evolution();//主体函数
void Initialize(Population& in_popu, uint in_length);//初始化个体和种群
void Initialize(Population& in_popu, uint in_length, double in_threshould);//初始化个体和种群
int Fitness(Individual* pIndi);//个体适应度
int PopuFitness(Population& Popu);//群体适应度
void Crossover(Population& Popu, double in_prob);//交叉算子
void Mutation(Population& Popu, double in_prob);//变异算子
void Select(Population& popu, Individual **array_indiv);//选择算子
double Probability();//返回0~1之间的概率值
void RandomPermutation(uint* in_pchrom, uint in_length);//随机排列
void Swap(uint& in_1, uint& in_2);//交换两个数的位置
void WeightMappingCrossover(uint* chrom1, uint* chrom2, uint in_length, uint in_pos);//WMX算子
void Sort(uint* in_original, uint in_length, uint* out_sorted, uint* out_index);
void Evaluate(Population& popu);
Individual* Copy(Individual* in_indi);



#endif // GENETICALGORITHM_H
