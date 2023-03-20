#include "operation.h"
#include "TaxiPlanningAlgorithm.h"
#include <assert.h>
#include <QTime>
#include "globalvariable.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <math.h>
#include <windows.h>

 uint g_generation = 30;
 double g_fitnessalias = 0.05;
 double g_mutationrate = 0.75;
 double g_crossoverrate = 0.6;
double g_ratio = 1e5;

void Evolution()
{
    /*random seed*/
    QTime time;
    time= QTime::currentTime();
    qsrand(time.msec()+time.second()*1000);
    /*cumpute the cost in join time order*/
    Individual indi;
    indi.m_length = g_objcount;
    indi.m_chromosome = new uint[indi.m_length];
    for(uint i=0;i<indi.m_length;i++){
        indi.m_chromosome[i] = i;
    }
    assert(Fitness(&indi) == 0);
    /*optimize the planning order*/
    Population popu(10);//种群大小
    Initialize(popu, g_objcount);//初始种群
    //Initialize(popu, g_objcount, indi.m_fitness);//带阈值的初始化
    double fitp = popu.m_popufitness;
    double fits;
    int count = 0;
    Individual** array_indiv = new Individual*[popu.m_number];
    uint generation;
    for(generation=0; generation<g_generation;generation++){
        Select(popu, array_indiv);
        Evaluate(popu);
        fits = popu.m_popufitness;
        if(fabs(fits-fitp)< g_fitnessalias){//0.05
            count++;
        }
        if(count >= 20){
            break;
        }
        fitp = fits;
        Crossover(popu, g_crossoverrate);//0.75
        Mutation(popu, g_mutationrate);//0.6
        //Sleep(1);
    }
    /*choose the best individual from the resulting population*/
    Individual* best = popu.m_indiv[0];
    for(uint i=1; i<popu.m_number; i++){
        if(best->m_fitness < popu.m_indiv[i]->m_fitness){
            best = popu.m_indiv[i];
        }
    }
    /*print relevant informations to file*/
    QFile file("..\\data_out\\GAExperiment.txt");
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << QString("failed to create GAExperiment output file");
    }
    else{
        QTextStream ts(&file);
        //put put basic parameters
        ts << "Basic setups:\n";
        ts << "generation=" << generation << "\n";
        ts << "crossover rate=" << g_crossoverrate << "\n";
        ts << "mutation rate=" << g_mutationrate << "\n";
        ts << "fitness alias=" << g_fitnessalias << "\n\n";

        //about the best individual
        ts << "Best planning order:\n";
        for(uint i=0; i<best->m_length;i++){
            ts << g_vehs.at(best->m_chromosome[i])->m_id << "  ";
        }
        ts << "\n" << "Total taxi time:\n";
        ts << g_ratio/best->m_fitness << "\n";

        //about the original sequential individual
        ts << "\n" << "Original taxi time:\n";
        ts << g_ratio/indi.m_fitness;

        file.close();
    }
    delete[] array_indiv;
    popu.Clear();

}
void  Select(Population& popu, Individual** array_indiv)
{
    double slice;
    Individual* pindi;
    double tmp = 0;

    uint count = 0;
    for(uint i=0; i<popu.m_number; i++){
        tmp = 0;
        slice = Probability()*popu.m_popufitness;
        for(uint j=0; j<popu.m_number; j++){
            pindi = popu.m_indiv[j];
            tmp += pindi->m_fitness;
            if(tmp >= slice){
                //indiv.push_back(Copy(pindi));
                array_indiv[i] = Copy(pindi);
                count++;
                break;
            }
        }
    }
    popu.Clear();
    assert(count == popu.m_number);
    for(uint i=0; i<popu.m_number;i++){
        //popu.m_indiv.push_back(indiv.at(i));
        popu.m_indiv[i] = array_indiv[i];
    }

}

Individual* Copy(Individual* in_indi)
{
    Individual* pindi = new Individual;
    pindi->m_length = in_indi->m_length;
    pindi->m_chromosome = new uint[pindi->m_length];
    pindi->m_fitness = in_indi->m_fitness;
    for(uint i=0;i<in_indi->m_length;i++){
        pindi->m_chromosome[i] = in_indi->m_chromosome[i];
    }
    return pindi;
}

void Evaluate(Population& popu)
{
    Individual* pindi;
    for(uint i=0; i<popu.m_number;i++){
        pindi = popu.m_indiv[i];
        Fitness(pindi);
    }
    PopuFitness(popu);
}


void  Initialize(Population& in_popu, uint in_length)
{
    Individual* pIndi;
    for(uint i=0;i<in_popu.m_number;i++){
        pIndi = new Individual;
        pIndi->m_length = in_length;
        pIndi->m_chromosome = new uint[in_length];
        RandomPermutation(pIndi->m_chromosome,in_length);//generate initial permutations randomly
        assert(Fitness(pIndi) == 0);
        in_popu.m_indiv[i] = pIndi;
    }
    PopuFitness(in_popu);
}

void Initialize(Population& in_popu, uint in_length, double in_threshould)
{
    Individual* pIndi;
    uint count = 0;
    while(true){
        if(count == in_popu.m_number){
            break;
        }
        pIndi = new Individual;
        pIndi->m_length = in_length;
        pIndi->m_chromosome = new uint[in_length];
        RandomPermutation(pIndi->m_chromosome,in_length);//generate initial permutations randomly
        assert(Fitness(pIndi) == 0);
        if(pIndi->m_fitness < in_threshould){
            continue;
        }
        in_popu.m_indiv[count] = pIndi;
        count++;
    }
    PopuFitness(in_popu);
}

void  RandomPermutation(uint* in_pchrom, uint in_length)
{
    for(uint i=0;i<in_length;i++){
        in_pchrom[i] = i;
    }
    for(uint i=0;i<in_length;i++){
        Swap(in_pchrom[i], in_pchrom[GetRandomNumberWithin(0,in_length-1)]);
    }
}

void  Swap(uint& in_1, uint& in_2)
{
    uint tmp = in_1;
    in_1 = in_2;
    in_2 = tmp;
}

int Fitness(Individual* pIndi){
    double cost;
    int returnvalue = PrioritizedPlan(pIndi->m_chromosome, pIndi->m_length, cost);
    if(returnvalue == 0){
        pIndi->m_fitness = g_ratio/cost;//适应度
    }
    return returnvalue;
}

int  PopuFitness(Population& Popu)
{
    Popu.m_popufitness = 0;
    for(uint i=0;i<Popu.m_number;i++){
        Popu.m_popufitness += Popu.m_indiv[i]->m_fitness;
    }
    return 0;
}

void    Crossover(Population& Popu, double in_prob)
{
    //先生成种群大小的随机序列，然后按顺序两两配对
    uint* randompair = new uint[Popu.m_number];
    RandomPermutation(randompair, Popu.m_number);
    double prob;
    uint pos;
    uint indi_length = Popu.m_indiv[0]->m_length;
    Individual* indiv1;
    Individual* indiv2;
    for(uint i=0;i<Popu.m_number;i += 2){//for each pair, do crossover accordinig to the probablity
        prob = Probability();
        if(prob < in_prob){//do crossover
            pos = GetRandomNumberWithin(1, indi_length - 1);
            indiv1 = Popu.m_indiv[randompair[i]];
            indiv2 = Popu.m_indiv[randompair[i+1]];
            WeightMappingCrossover(indiv1->m_chromosome, indiv2->m_chromosome,indi_length,pos);
        }
    }
    delete[] randompair;
}

double  Probability(){
    return (double)qrand()/(RAND_MAX+1);
}

void WeightMappingCrossover(uint* chrom1, uint* chrom2, uint in_length, uint in_pos)
{
    uint sort_length = in_length - in_pos;
    uint* array_sorted1 = new uint[sort_length];
    uint* array_index1 = new uint[sort_length];
    uint* array_sorted2 = new uint[sort_length];
    uint* array_index2 = new uint[sort_length];
    Sort(chrom1+in_pos, sort_length, array_sorted1, array_index1);
    Sort(chrom2+in_pos, sort_length, array_sorted2, array_index2);
    for(uint i=0; i<sort_length ;i++){
        chrom1[in_pos+i] = array_sorted1[array_index2[i]];
        chrom2[in_pos+i] = array_sorted2[array_index1[i]];
    }
    delete[] array_index1;
    delete[] array_index2;
    delete[] array_sorted1;
    delete[] array_sorted2;
}

void Sort(uint* in_original, uint in_length, uint* out_sorted, uint* out_index)
{
    for(uint i=0;i<in_length;i++){
        out_sorted[i] = in_original[i];
        out_index[i]=i;
    }
    int count;
    for(uint i=0; i< in_length; i++){
        count = 0;
        for(uint j=0;j<in_length;j++){
            if(in_original[i] > in_original[j]){
                count++;
            }
        }
        out_index[i] = count;
        out_sorted[count] = in_original[i];
    }
}

void    Mutation(Population& Popu, double in_prob)
{
    double prob;
    uint pos1, pos2;
    uint length = Popu.m_indiv[0]->m_length;
    Individual* pindi;
    for(uint i=0; i<Popu.m_number; i++){
        prob = Probability();
        if(prob < in_prob){
            pindi = Popu.m_indiv[i];
            pos1 = GetRandomNumberWithin(0,length - 1);
            pos2 = GetRandomNumberWithin(0,length - 1);
            if(pos1 != pos2){
                Swap(pindi->m_chromosome[pos1], pindi->m_chromosome[pos2]);
            }
        }
    }
}
