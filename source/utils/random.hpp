#ifndef    RANDOM_HPP
#define    RANDOM_HPP
#include <memory>
#include <random>
template <typename RandomGenerator=std::mt19937>
class Random{
    std::unique_ptr<RandomGenerator> rnd;
    public:
    Random(int seed = 0){
        set_seed(seed);
    }
    void set_seed(int seed){
        rnd = std::unique_ptr<RandomGenerator>(new RandomGenerator(seed));
    }
    double gauss(double mean, double stev){
        std::normal_distribution<> g(mean, stev);
        return g(rnd);
    }
    double uniform(void){
        std::uniform_real_distribution<> g(0.0, 1.0);
        return g(rnd);
    }
    double uniform(double begin, double end){
        std::uniform_real_distribution<> g(begin, end);
        return g(rnd);
    }
    int uniform(int begin, int end){
        std::uniform_int_distribution<> g(begin, end);
        return g(rnd);
    }
};

#endif   //RANDOM_HPP
