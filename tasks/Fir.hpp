/* ----------------------------------------------------------------------------
 * avalon/orogen/uw_particle_localization/Fir.hpp
 * written by Christoph Mueller, Oct 2011
 * University of Bremen
 * ----------------------------------------------------------------------------
*/

#ifndef UW_LOCALIZATION__FIR_HPP
#define UW_LOCALIZATION__FIR_HPP

namespace uw_localization {

std::vector<double> MovingAverage(unsigned window_size) {
    std::vector<double> weights;

    for(unsigned i = 0; i < window_size; i++)
        weights.push_back(1.0 / window_size);

    return weights;
}

template<typename T>
T Fir(const std::vector<double>& w, const std::list<T>& samples) {
    T result = w.front() * samples.front();

    typename std::list<T>::const_iterator it = samples.begin();
    unsigned i = 1;

    for(it = ++samples.begin(); it != samples.end(); ++it) {
        result += w[i] * (*it);
        ++i;
    }

    return result;
}


}

#endif // UW_LOCALIZATION__FIR_HPP

