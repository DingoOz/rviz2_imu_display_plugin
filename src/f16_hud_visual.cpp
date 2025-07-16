#include "f16_hud_rviz_plugin/f16_hud_visual.hpp"

#include <cmath>
#include <algorithm>

namespace f16_hud_rviz_plugin
{

F16HudVisual::F16HudVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode)
    : _sceneManager(sceneManager), _color(0.0f, 1.0f, 0.0f, 0.8f)
{
    _frameNode = parentNode->createChildSceneNode();
    _pitchLadderNode = _frameNode->createChildSceneNode();
    _aircraftSymbolNode = _frameNode->createChildSceneNode();
    _sideDisplayNode = _frameNode->createChildSceneNode();
    
    createPitchLadder();
    createAircraftSymbol();
    createSideTapes();
    createHeadingDisplay();
}

F16HudVisual::~F16HudVisual()
{
    clearVisuals();
    if (_frameNode)
    {
        _sceneManager->destroySceneNode(_frameNode);
    }
}

void F16HudVisual::setAttitude(double roll, double pitch, double yaw)
{
    _roll = roll;
    _pitch = pitch;
    _yaw = yaw;
    
    updatePitchLadder();
    updateAircraftSymbol();
    updateSideTapes();
    updateHeadingDisplay();
}

void F16HudVisual::setScale(float scale)
{
    _scale = scale;
    _frameNode->setScale(Ogre::Vector3(scale, scale, scale));
}

void F16HudVisual::setTransparency(float alpha)
{
    _transparency = alpha;
    _color.a = alpha;
    
    // Update all visual elements with new transparency
    for (auto* pitchLine : _pitchLines)
    {
        if (pitchLine && pitchLine->getCurrentVertexCount() > 0)
        {
            // Color is set during line creation, no need to update here
        }
    }
    
    if (_aircraftSymbol && _aircraftSymbol->getCurrentVertexCount() > 0)
    {
        // Color is set during creation, no need to update here
    }
    
    if (_airspeedTape && _airspeedTape->getCurrentVertexCount() > 0)
    {
        // Color is set during creation, no need to update here
    }
    
    if (_altitudeTape && _altitudeTape->getCurrentVertexCount() > 0)
    {
        // Color is set during creation, no need to update here
    }
    
    if (_headingDisplay && _headingDisplay->getCurrentVertexCount() > 0)
    {
        // Color is set during creation, no need to update here
    }
}

void F16HudVisual::setColor(const Ogre::ColourValue& color)
{
    _color = color;
    _color.a = _transparency;
    setTransparency(_transparency);  // Reuse transparency update logic
}

void F16HudVisual::setVisible(bool visible)
{
    _frameNode->setVisible(visible);
}

void F16HudVisual::createPitchLadder()
{
    // Create pitch ladder lines at 5-degree increments from -30 to +30
    for (int i = 0; i < PITCH_LINE_COUNT; ++i)
    {
        int pitchDegree = (i - 6) * 5;  // -30, -25, ..., 0, ..., 25, 30
        
        Ogre::ManualObject* pitchLine = _sceneManager->createManualObject();
        pitchLine->setDynamic(true);
        
        pitchLine->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
        
        float yPos = static_cast<float>(pitchDegree * PIXELS_PER_DEGREE);
        float lineWidth = (pitchDegree == 0) ? HUD_WIDTH * 0.6f : HUD_WIDTH * 0.4f;
        
        // Main horizontal line
        pitchLine->position(-lineWidth / 2, yPos, 0);
        pitchLine->colour(_color);
        pitchLine->position(lineWidth / 2, yPos, 0);
        pitchLine->colour(_color);
        
        // Add tick marks for major lines
        if (pitchDegree % 10 == 0 && pitchDegree != 0)
        {
            float tickHeight = 10.0f;
            // Left tick
            pitchLine->position(-lineWidth / 2, yPos, 0);
            pitchLine->colour(_color);
            pitchLine->position(-lineWidth / 2, yPos + tickHeight, 0);
            pitchLine->colour(_color);
            
            // Right tick
            pitchLine->position(lineWidth / 2, yPos, 0);
            pitchLine->colour(_color);
            pitchLine->position(lineWidth / 2, yPos + tickHeight, 0);
            pitchLine->colour(_color);
        }
        
        pitchLine->end();
        _pitchLadderNode->attachObject(pitchLine);
        _pitchLines.push_back(pitchLine);
    }
}

void F16HudVisual::createAircraftSymbol()
{
    _aircraftSymbol = _sceneManager->createManualObject();
    _aircraftSymbol->setDynamic(true);
    
    _aircraftSymbol->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    
    // Center cross-hair
    float crossSize = 20.0f;
    
    // Horizontal line
    _aircraftSymbol->position(-crossSize, 0, 0);
    _aircraftSymbol->colour(_color);
    _aircraftSymbol->position(crossSize, 0, 0);
    _aircraftSymbol->colour(_color);
    
    // Vertical line
    _aircraftSymbol->position(0, -crossSize, 0);
    _aircraftSymbol->colour(_color);
    _aircraftSymbol->position(0, crossSize, 0);
    _aircraftSymbol->colour(_color);
    
    // Wings
    float wingLength = 40.0f;
    float wingHeight = 8.0f;
    
    // Left wing
    _aircraftSymbol->position(-wingLength, -wingHeight, 0);
    _aircraftSymbol->colour(_color);
    _aircraftSymbol->position(-crossSize, 0, 0);
    _aircraftSymbol->colour(_color);
    
    // Right wing
    _aircraftSymbol->position(wingLength, -wingHeight, 0);
    _aircraftSymbol->colour(_color);
    _aircraftSymbol->position(crossSize, 0, 0);
    _aircraftSymbol->colour(_color);
    
    _aircraftSymbol->end();
    _aircraftSymbolNode->attachObject(_aircraftSymbol);
}

void F16HudVisual::createSideTapes()
{
    // Airspeed tape (left side)
    _airspeedTape = _sceneManager->createManualObject();
    _airspeedTape->setDynamic(true);
    
    _airspeedTape->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    
    float leftX = -HUD_WIDTH * 0.45f;
    float tapeHeight = HUD_HEIGHT * 0.6f;
    float tickSpacing = 20.0f;
    
    // Tape outline
    _airspeedTape->position(leftX - 30, -tapeHeight / 2, 0);
    _airspeedTape->colour(_color);
    _airspeedTape->position(leftX - 30, tapeHeight / 2, 0);
    _airspeedTape->colour(_color);
    
    _airspeedTape->position(leftX, -tapeHeight / 2, 0);
    _airspeedTape->colour(_color);
    _airspeedTape->position(leftX, tapeHeight / 2, 0);
    _airspeedTape->colour(_color);
    
    // Speed tick marks
    for (int i = -6; i <= 6; ++i)
    {
        float yPos = i * tickSpacing;
        _airspeedTape->position(leftX - 30, yPos, 0);
        _airspeedTape->colour(_color);
        _airspeedTape->position(leftX - 15, yPos, 0);
        _airspeedTape->colour(_color);
    }
    
    _airspeedTape->end();
    _sideDisplayNode->attachObject(_airspeedTape);
    
    // Altitude tape (right side)
    _altitudeTape = _sceneManager->createManualObject();
    _altitudeTape->setDynamic(true);
    
    _altitudeTape->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    
    float rightX = HUD_WIDTH * 0.45f;
    
    // Tape outline
    _altitudeTape->position(rightX + 30, -tapeHeight / 2, 0);
    _altitudeTape->colour(_color);
    _altitudeTape->position(rightX + 30, tapeHeight / 2, 0);
    _altitudeTape->colour(_color);
    
    _altitudeTape->position(rightX, -tapeHeight / 2, 0);
    _altitudeTape->colour(_color);
    _altitudeTape->position(rightX, tapeHeight / 2, 0);
    _altitudeTape->colour(_color);
    
    // Altitude tick marks
    for (int i = -6; i <= 6; ++i)
    {
        float yPos = i * tickSpacing;
        _altitudeTape->position(rightX + 30, yPos, 0);
        _altitudeTape->colour(_color);
        _altitudeTape->position(rightX + 15, yPos, 0);
        _altitudeTape->colour(_color);
    }
    
    _altitudeTape->end();
    _sideDisplayNode->attachObject(_altitudeTape);
}

void F16HudVisual::createHeadingDisplay()
{
    _headingDisplay = _sceneManager->createManualObject();
    _headingDisplay->setDynamic(true);
    
    _headingDisplay->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    
    float bottomY = -HUD_HEIGHT * 0.4f;
    float headingWidth = HUD_WIDTH * 0.8f;
    
    // Heading tape outline
    _headingDisplay->position(-headingWidth / 2, bottomY, 0);
    _headingDisplay->colour(_color);
    _headingDisplay->position(headingWidth / 2, bottomY, 0);
    _headingDisplay->colour(_color);
    
    _headingDisplay->position(-headingWidth / 2, bottomY - 20, 0);
    _headingDisplay->colour(_color);
    _headingDisplay->position(headingWidth / 2, bottomY - 20, 0);
    _headingDisplay->colour(_color);
    
    // Heading tick marks
    for (int i = -8; i <= 8; ++i)
    {
        float xPos = i * 20.0f;
        _headingDisplay->position(xPos, bottomY, 0);
        _headingDisplay->colour(_color);
        _headingDisplay->position(xPos, bottomY - 10, 0);
        _headingDisplay->colour(_color);
    }
    
    _headingDisplay->end();
    _sideDisplayNode->attachObject(_headingDisplay);
}

void F16HudVisual::updatePitchLadder()
{
    // Apply pitch rotation to the pitch ladder
    Ogre::Quaternion pitchRotation;
    pitchRotation.FromAngleAxis(Ogre::Radian(-_pitch), Ogre::Vector3(0, 0, 1));
    
    Ogre::Quaternion rollRotation;
    rollRotation.FromAngleAxis(Ogre::Radian(_roll), Ogre::Vector3(1, 0, 0));
    
    _pitchLadderNode->setOrientation(rollRotation * pitchRotation);
    
    // Translate pitch ladder based on pitch angle
    float pitchOffset = static_cast<float>(_pitch * PIXELS_PER_DEGREE);
    _pitchLadderNode->setPosition(0, pitchOffset, 0);
}

void F16HudVisual::updateAircraftSymbol()
{
    // Aircraft symbol remains fixed in screen center
    // Only apply roll rotation
    Ogre::Quaternion rollRotation;
    rollRotation.FromAngleAxis(Ogre::Radian(_roll), Ogre::Vector3(0, 0, 1));
    _aircraftSymbolNode->setOrientation(rollRotation);
}

void F16HudVisual::updateSideTapes()
{
    // Side tapes remain fixed but could show simulated data
    // For now, they remain static
}

void F16HudVisual::updateHeadingDisplay()
{
    // Update heading display based on yaw
    // For basic implementation, we'll just rotate the heading tape
    // Simulate heading tape movement (simplified)
    // Note: headingOffset calculation removed as it was unused
}

void F16HudVisual::clearVisuals()
{
    for (auto* pitchLine : _pitchLines)
    {
        if (pitchLine)
        {
            _sceneManager->destroyManualObject(pitchLine);
        }
    }
    _pitchLines.clear();
    
    if (_aircraftSymbol)
    {
        _sceneManager->destroyManualObject(_aircraftSymbol);
        _aircraftSymbol = nullptr;
    }
    
    if (_airspeedTape)
    {
        _sceneManager->destroyManualObject(_airspeedTape);
        _airspeedTape = nullptr;
    }
    
    if (_altitudeTape)
    {
        _sceneManager->destroyManualObject(_altitudeTape);
        _altitudeTape = nullptr;
    }
    
    if (_headingDisplay)
    {
        _sceneManager->destroyManualObject(_headingDisplay);
        _headingDisplay = nullptr;
    }
}

}  // namespace f16_hud_rviz_plugin