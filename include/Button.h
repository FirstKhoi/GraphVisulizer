//
//  Button.h
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#pragma once
#include <SFML/Graphics.hpp>
#include <string>
#include <functional>

class Button {
public:
    Button(float x, float y, float width, float height, std::string text, sf::Font& font);

    void render(sf::RenderWindow& window);
    bool isClicked(float mouseX, float mouseY);
    void update(float mouseX, float mouseY);
    
    void setEnabled(bool enabled);
    void setSelected(bool selected);
    void setText(const std::string& text);
    
    std::function<void()> onClick;

private:
    sf::RectangleShape shape;
    sf::Text label;
    sf::Font* fontPtr;
    
    sf::Color baseColor;
    sf::Color hoverColor;
    sf::Color disabledColor;
    sf::Color selectedColor;
    
    bool isHovered = false;
    bool isEnabled = true;
    bool isSelected = false;
    
    float x, y, width, height;
};
