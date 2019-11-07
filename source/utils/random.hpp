#ifndef    RANDOM_HPP
#define    RANDOM_HPP
#include <memory>
#include <random>
#include <iterator>
#include <algorithm>
template <typename RandomGenerator=std::mt19937>
class Random{
    std::shared_ptr<RandomGenerator> rnd;
    public:
    Random(int seed = 0){
        set_seed(seed);
    }
    void set_seed(void){
        std::random_device rng;
        rnd = std::unique_ptr<RandomGenerator>(new RandomGenerator(rng()));
    }
    void set_seed(int seed){
        rnd = std::unique_ptr<RandomGenerator>(new RandomGenerator(seed));
    }
    double gauss(void){
        std::normal_distribution<> g(0.0, 1.0);
        return g(rnd);
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
    template <class RandomAccessIterator>
    void shuffle(RandomAccessIterator first, RandomAccessIterator last){
        std::shuffle(first, last, *rnd);
    }

};

#endif   //RANDOM_HPP
