/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <dvo_core/dense_tracking.h>

DenseTracker::Config::Config() :
  FirstLevel(3),
  LastLevel(1),
  MaxIterationsPerLevel(100),
  Precision(5e-7),
  Lambda(0),
  UseInitialEstimate(false),
  UseWeighting(true),
  Mu(0),
  InfluenceFuntionType(InfluenceFunctions::TDistribution),
  InfluenceFunctionParam(TDistributionInfluenceFunction::DEFAULT_DOF),
  ScaleEstimatorType(ScaleEstimators::TDistribution),
  ScaleEstimatorParam(TDistributionScaleEstimator::DEFAULT_DOF)
{
}

size_t DenseTracker::Config::getNumLevels() const
{
  return FirstLevel + 1;
}

bool DenseTracker::Config::UseTemporalSmoothing() const
{
  return Lambda > 1e-6;
}

bool DenseTracker::Config::UseEstimateSmoothing() const
{
  return Mu > 1e-6;
}

bool DenseTracker::Config::IsSane() const
{
  return FirstLevel >= LastLevel;
}

DenseTracker::IterationContext::IterationContext(const Config& cfg) :
    cfg(cfg)
{
}

bool DenseTracker::IterationContext::IsFirstIteration() const
{
  return IsFirstLevel() && IsFirstIterationOnLevel();
}

bool DenseTracker::IterationContext::IsFirstIterationOnLevel() const
{
  return Iteration == 0;
}

bool DenseTracker::IterationContext::IsFirstLevel() const
{
  return cfg.FirstLevel == Level;
}

bool DenseTracker::IterationContext::IsLastLevel() const
{
  return cfg.LastLevel == Level;
}

double DenseTracker::IterationContext::ErrorDiff() const
{
  return LastError - Error;
}

bool DenseTracker::IterationContext::IterationsExceeded() const
{
  int max_iterations = cfg.MaxIterationsPerLevel;

  return Iteration >= max_iterations;
}


std::ostream& operator<< (std::ostream& out, DenseTracker::Config& config)
{
  out
  << "First Level = " << config.FirstLevel
  << ", Last Level = " << config.LastLevel
  << ", Max Iterations per Level = " << config.MaxIterationsPerLevel
  << ", Precision = " << config.Precision
  << ", Lambda = " << config.Lambda
  << ", Mu = " << config.Mu
  << ", Use Initial Estimate = " << (config.UseInitialEstimate ? "true" : "false")
  << ", Use Weighting = " << (config.UseWeighting ? "true" : "false")
  << ", Scale Estimator = " << ScaleEstimators::str(config.ScaleEstimatorType)
  << ", Scale Estimator Param = " << config.ScaleEstimatorParam
  << ", Influence Function = " << InfluenceFunctions::str(config.InfluenceFuntionType)
  << ", Influence Function Param = " << config.InfluenceFunctionParam
  ;

  return out;
}

std::ostream& operator <<(std::ostream& out, DenseTracker::IterationContext& ctx)
{
  out
  << "Level: " << ctx.Level
  << ", Iteration: " << ctx.Iteration
  << ", LastError: " << ctx.LastError
  << ", Error: " << ctx.Error
  << ", ErrorDiff: " << ctx.ErrorDiff()
  ;

  return out;
}


