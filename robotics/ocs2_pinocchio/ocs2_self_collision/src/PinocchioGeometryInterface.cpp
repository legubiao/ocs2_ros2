/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
#include <pinocchio/fwd.hpp>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/collision/distance.hpp>

#include <sstream>

namespace ocs2
{
    PinocchioGeometryInterface::PinocchioGeometryInterface(
        const PinocchioInterface& pinocchioInterface,
        const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
        : geometryModelPtr_(std::make_shared<pinocchio::GeometryModel>())
    {
        buildGeomFromPinocchioInterface(pinocchioInterface, *geometryModelPtr_);

        addCollisionObjectPairs(collisionObjectPairs);
    }

    PinocchioGeometryInterface::PinocchioGeometryInterface(
        const PinocchioInterface& pinocchioInterface,
        const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
        const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
        : geometryModelPtr_(std::make_shared<pinocchio::GeometryModel>())
    {
        buildGeomFromPinocchioInterface(pinocchioInterface, *geometryModelPtr_);

        addCollisionObjectPairs(collisionObjectPairs);
        addCollisionLinkPairs(pinocchioInterface, collisionLinkPairs);
    }

    PinocchioGeometryInterface::PinocchioGeometryInterface(
        const PinocchioInterface& pinocchioInterface,
        const std::string& urdf_path,
        const std::vector<std::pair<std::string, std::string>>&
        collisionLinkPairs,
        const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs):
        geometryModelPtr_(std::make_shared<pinocchio::GeometryModel>())
    {
        buildGeomFromUrdf(pinocchioInterface, urdf_path, *geometryModelPtr_);
        addCollisionObjectPairs(collisionObjectPairs);
        addCollisionLinkPairs(pinocchioInterface, collisionLinkPairs);
    }


    std::vector<ocs2::collision::DistanceResult>
    PinocchioGeometryInterface::computeDistances(
        const PinocchioInterface& pinocchioInterface) const
    {
        pinocchio::GeometryData geometryData(*geometryModelPtr_);

        updateGeometryPlacements(pinocchioInterface.getModel(),
                                 pinocchioInterface.getData(),
                                 *geometryModelPtr_, geometryData);
        pinocchio::computeDistances(*geometryModelPtr_, geometryData);

        return std::move(geometryData.distanceResults);
    }


    size_t PinocchioGeometryInterface::getNumCollisionPairs() const
    {
        return geometryModelPtr_->collisionPairs.size();
    }


    void PinocchioGeometryInterface::buildGeomFromPinocchioInterface(
        const PinocchioInterface& pinocchioInterface,
        pinocchio::GeometryModel& geomModel)
    {
        // Replay the original URDF XML that was used to build pinocchioInterface
        // straight into pinocchio::urdf::buildGeom. Pre-urdfdom-5 we used to
        // round-trip via urdf::exportURDF(ModelInterface), but that API was
        // removed in urdfdom 5.0 (Lyrical+), so the raw XML is now the only
        // stable bridge. Construct the PinocchioInterface via the
        // getPinocchioInterfaceFromUrdf{File,String} helpers to guarantee
        // getUrdfXmlString() is populated.
        const std::string& urdfXml = pinocchioInterface.getUrdfXmlString();
        if (urdfXml.empty())
        {
            throw std::runtime_error(
                "PinocchioGeometryInterface: cannot rebuild GeometryModel because "
                "the PinocchioInterface was not constructed from a URDF source "
                "(empty getUrdfXmlString()). Build it via "
                "getPinocchioInterfaceFromUrdfFile/String, or use the "
                "PinocchioGeometryInterface(pinocchioInterface, urdf_path, ...) "
                "overload that reads the URDF file directly.");
        }

        std::istringstream urdfAsStringStream(urdfXml);
        pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfAsStringStream,
                                   pinocchio::COLLISION, geomModel);
    }

    void PinocchioGeometryInterface::buildGeomFromUrdf(
        const PinocchioInterface& pinocchioInterface,
        const std::string& urdf_path,
        pinocchio::GeometryModel& geomModel)
    {
        std::ifstream file(urdf_path);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open file: " + urdf_path);
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        // std::cout << "urdf content: " << buffer.str() << std::endl;

        pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), buffer,
                                   pinocchio::COLLISION, geomModel);
    }


    void PinocchioGeometryInterface::addCollisionObjectPairs(
        const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs) const
    {
        for (const auto& pair : collisionObjectPairs)
        {
            geometryModelPtr_->addCollisionPair(
                pinocchio::CollisionPair{pair.first, pair.second});
        }
    }


    void PinocchioGeometryInterface::addCollisionLinkPairs(
        const PinocchioInterface& pinocchioInterface,
        const std::vector<std::pair<std::string, std::string>>&
        collisionLinkPairs) const
    {
        for (const auto& linkPair : collisionLinkPairs)
        {
            bool addedPair = false;
            for (size_t i = 0; i < geometryModelPtr_->geometryObjects.size(); ++i)
            {
                const pinocchio::GeometryObject& object1 =
                    geometryModelPtr_->geometryObjects[i];
                const std::string parentFrameName1 =
                    pinocchioInterface.getModel().frames[object1.parentFrame].name;
                if (parentFrameName1 == linkPair.first)
                {
                    for (size_t j = 0; j < geometryModelPtr_->geometryObjects.size(); ++j)
                    {
                        const pinocchio::GeometryObject& object2 =
                            geometryModelPtr_->geometryObjects[j];
                        const std::string parentFrameName2 =
                            pinocchioInterface.getModel().frames[object2.parentFrame].name;
                        if (parentFrameName2 == linkPair.second)
                        {
                            geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{i, j});
                            addedPair = true;
                        }
                    }
                }
            }
            if (!addedPair)
            {
                std::cerr << "WARNING: in collision link pair [" << linkPair.first << ", "
                    << linkPair.second
                    << "], one or both of the links don't exist in the "
                    "pinocchio/urdf model\n";
            }
        }
    }
} // namespace ocs2
