/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>

#include <ompl/config.h>
#include <../tests/resources/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

class Plane2DEnvironment
{
public:

    Plane2DEnvironment(const char *ppm_file)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            lightning_.reset(new ot::Lightning(ob::StateSpacePtr(space)));
            lightning_->setFilePath("lightning.db");
            // set state validity checking for this space
            lightning_->setStateValidityChecker(boost::bind(&Plane2DEnvironment::isStateValid, this, _1));
            space->setup();
            lightning_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            vss_ = lightning_->getSpaceInformation()->allocValidStateSampler();
        }
    }

    ~Plane2DEnvironment()
    {
        lightning_->save();
    }

    bool plan()
    {
        if (!lightning_)
            return false;
        ob::ScopedState<> start(lightning_->getStateSpace());
        vss_->sample(start.get());
        ob::ScopedState<> goal(lightning_->getStateSpace());
        vss_->sample(goal.get());
        lightning_->setStartAndGoalStates(start, goal);
        bool solved = lightning_->solve(10.);
        if (solved)
            OMPL_INFORM("Found solution in %g seconds",
                lightning_->getLastPlanComputationTime());
        else
            OMPL_INFORM("No solution found");
        return false;
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    ot::LightningPtr lightning_;
    ob::ValidStateSamplerPtr vss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
};

int main(int, char **)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    boost::filesystem::path path(TEST_RESOURCES_DIR);
    Plane2DEnvironment env((path / "ppm/floor.ppm").string().c_str());

    for (unsigned int i=0; i<100; ++i)
        env.plan();

    return 0;
}
