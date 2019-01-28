/*
 * AMZ-Driverless
 * Copyright (c) 2018 Authors:
 *   - Juraj Kabzan <kabzanj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Noises
#include "noise.hpp"

// STD Includes
#include <random>
#include <ctime>
#include <chrono>

namespace gazebo {
namespace noise {

Eigen::Vector2d gaussiaNoise2D(double mu, double sigma) {
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables
    // see wikipedia
    double U = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    double V = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    double X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
    double Y = sqrt(-2.0 * ::log(U)) * sin(2.0 * M_PI * V); // the other indep. normal variable
    // we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    Y = sigma * Y + mu;
    return Eigen::Vector2d(X, Y);
}

Eigen::Vector3d gaussiaNoise3D(double mu, double sigma) {
    const auto vect_2d = gaussiaNoise2D(mu, sigma);
    return Eigen::Vector3d(vect_2d.x(), vect_2d.y(), 0.0);
}

double getGaussianNoise(double mean, double var) {
    std::normal_distribution<double> distribution(mean, var);
    // construct a trivial random generator engine from a time-based seed:
    long                             seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine       generator(seed);
    return distribution(generator);
}

bool probability(const double likelihood, double &lik_res) {
    const double rand = (double) std::rand() / (double) RAND_MAX; // normalized uniform random variable
    lik_res = std::max(likelihood, rand);
    return rand <= likelihood;
}

}  // namespace noise
}  // namespace gazebo
