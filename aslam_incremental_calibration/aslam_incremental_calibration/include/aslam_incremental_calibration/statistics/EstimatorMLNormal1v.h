/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file EstimatorMLNormal1v.h
    \brief This file implements an ML estimator for univariate normal
           distributions.
  */

#include <vector>

#include "aslam_incremental_calibration/statistics/NormalDistribution.h"
#include "aslam_incremental_calibration/base/Serializable.h"

namespace aslam {
  namespace calibration {

    /** The class EstimatorML is implemented for univariate normal
        distributions.
        \brief Univariate normal distribution ML estimator
      */
    template <> class EstimatorML<NormalDistribution<1> > :
      public virtual Serializable {
    public:
      /** \name Types definitions
        @{
        */
      /// Point type
      typedef NormalDistribution<1>::RandomVariable Point;
      /// Points container
      typedef std::vector<Point> Container;
      /// Constant point iterator
      typedef Container::const_iterator ConstPointIterator;
      /** @}
        */

      /** \name Constructors/destructor
        @{
        */
      /// Default constructor
      EstimatorML();
      /// Copy constructor
      EstimatorML(const EstimatorML& other);
      /// Assignment operator
      EstimatorML& operator = (const EstimatorML& other);
      /// Destructor
      virtual ~EstimatorML();
      /** @}
        */

      /** \name Accessors
        @{
        */
      /// Returns the number of points
      size_t getNumPoints() const;
      /// Returns the validity state of the estimator
      bool getValid() const;
      /// Returns the estimated distribution
      const NormalDistribution<1>& getDistribution() const;
      /// Add a point to the estimator
      void addPoint(const Point& point);
      /// Add points to the estimator
      void addPoints(const ConstPointIterator& itStart,
        const ConstPointIterator& itEnd);
      /// Add points to the estimator with responsibilities
      void addPoints(const ConstPointIterator& itStart,
        const ConstPointIterator& itEnd,
        const Eigen::Matrix<double, Eigen::Dynamic, 1>& responsibilities);
      /// Add points to the estimator
      void addPoints(const Container& points);
      /// Reset the estimator
      void reset();
      /** @}
        */

    protected:
      /** \name Stream methods
        @{
        */
      /// Reads from standard input
      virtual void read(std::istream& stream);
      /// Writes to standard output
      virtual void write(std::ostream& stream) const;
      /// Reads from a file
      virtual void read(std::ifstream& stream);
      /// Writes to a file
      virtual void write(std::ofstream& stream) const;
      /** @}
        */

      /** \name Protected members
        @{
        */
      /// Estimated distribution
      NormalDistribution<1> mDistribution;
      /// Number of points in the estimator
      size_t mNumPoints;
      /// Valid flag
      bool mValid;
      /// Sum of the values
      double mValuesSum;
      /// Squared sum of the values
      double mSquaredValuesSum;
      /** @}
        */

    };

  }
}
