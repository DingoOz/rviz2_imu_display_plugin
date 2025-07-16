#pragma once

#include <memory>
#include <vector>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreColourValue.h>

namespace f16_hud_rviz_plugin
{

class F16HudVisual
{
public:
    F16HudVisual(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode);
    ~F16HudVisual();
    
    void setAttitude(double roll, double pitch, double yaw);
    void setScale(float scale);
    void setTransparency(float alpha);
    void setColor(const Ogre::ColourValue& color);
    
    void setVisible(bool visible);

private:
    void createPitchLadder();
    void createAircraftSymbol();
    void createSideTapes();
    void createHeadingDisplay();
    
    void updatePitchLadder();
    void updateAircraftSymbol();
    void updateSideTapes();
    void updateHeadingDisplay();
    
    void clearVisuals();
    
    // Ogre rendering objects
    Ogre::SceneManager* _sceneManager;
    Ogre::SceneNode* _frameNode;
    Ogre::SceneNode* _pitchLadderNode;
    Ogre::SceneNode* _aircraftSymbolNode;
    Ogre::SceneNode* _sideDisplayNode;
    
    // HUD elements
    std::vector<Ogre::ManualObject*> _pitchLines;
    Ogre::ManualObject* _aircraftSymbol;
    Ogre::ManualObject* _airspeedTape;
    Ogre::ManualObject* _altitudeTape;
    Ogre::ManualObject* _headingDisplay;
    
    // Current state
    double _roll = 0.0;
    double _pitch = 0.0;
    double _yaw = 0.0;
    float _scale = 1.0f;
    float _transparency = 0.8f;
    Ogre::ColourValue _color;
    
    // Constants
    static constexpr double PITCH_DEGREES_VISIBLE = 60.0;
    static constexpr double PIXELS_PER_DEGREE = 5.0;
    static constexpr int PITCH_LINE_COUNT = 13;  // -30 to +30 in 5 degree increments
    static constexpr float HUD_WIDTH = 400.0f;
    static constexpr float HUD_HEIGHT = 300.0f;
};

}  // namespace f16_hud_rviz_plugin