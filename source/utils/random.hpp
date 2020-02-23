#ifndef    RANDOM_HPP
#define    RANDOM_HPP
#include <memory>
#include <random>
#include <iterator>
#include <algorithm>
namespace teyo_utils{

template <typename RandomGenerator=std::mt19937>
class Random{
    std::shared_ptr<RandomGenerator> rnd;
    int now_seed;
    public:
    Random(int seed = 0){
        set_seed(seed);
    }
    inline void set_seed(void){
        std::random_device rng;
        now_seed = rng();
        rnd = std::shared_ptr<RandomGenerator>(new RandomGenerator(now_seed));
    }
    inline void set_seed(int seed){
        now_seed = seed;
        rnd = std::shared_ptr<RandomGenerator>(new RandomGenerator(seed));
    }
    inline double gauss(void){
        std::normal_distribution<> g(0.0, 1.0);
        return g(*rnd);
    }
    inline double gauss(double mean, double stev){
        std::normal_distribution<> g(mean, stev);
        return g(*rnd);
    }
    inline double cauchy(void){
        std::cauchy_distribution<> g(0.0, 1.0);
        return g(*rnd);
    }
    inline double cauchy(double a, double b){
        std::cauchy_distribution<> g(a, b);
        return g(*rnd);
    }
    inline double uniform(void){
        std::uniform_real_distribution<> g(0.0, 1.0);
        return g(*rnd);
    }
    inline double uniform(double begin, double end){
        std::uniform_real_distribution<> g(begin, end);
        return g(*rnd);
    }
    inline int uniform(int begin, int end){
        std::uniform_int_distribution<> g(begin, end);
        return g(*rnd);
    }
    template <class RandomAccessIterator>
    inline void shuffle(RandomAccessIterator first, RandomAccessIterator last){
        std::shuffle(first, last, *rnd);
    }
    inline std::shared_ptr<RandomGenerator> get_generator(void){
        return rnd;
    }
    template <class RealType>
    inline double operator()(std::vector<RealType>& probabilities){
        std::discrete_distribution<std::size_t> dist(
                probabilities.begin(),
                probabilities.end()
        );
        return dist(*rnd);
    }
    inline double operator()(){
        return this->uniform(0.0, 1.0);
    }

    inline double operator()(double b){
        return this->uniform(0.0, b);
    }
    inline double operator()(double a, double b){
        return this->uniform(a, b);
    }

    inline int get_seed() { return now_seed; }
};
}
extern teyo_utils::Random<> rnd;
#endif   //RANDOM_HPP

