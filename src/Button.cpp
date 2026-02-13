//
//  Button.cpp
//  Graph Visualization - Data Structures & Algorithms Learning Tool
//
//  Created by Luong Nhat Khoi on 1/12/25.
//

#include "Button.h"

Button::Button(float x, float y, float w, float h, std::string t, sf::Font& font)
    : x(x), y(y), width(w), height(h), label(font, t), fontPtr(&font)
{
    shape.setPosition({x, y});
    shape.setSize({w, h});
    
    baseColor = sf::Color(70, 70, 70);
    hoverColor = sf::Color(100, 100, 100);
    disabledColor = sf::Color(50, 50, 50);
    selectedColor = sf::Color(0, 120, 200);
    
    shape.setFillColor(baseColor);
    shape.setOutlineColor(sf::Color::White);
    shape.setOutlineThickness(1.0f);

    label.setCharacterSize(14);
    label.setFillColor(sf::Color::White);

    sf::FloatRect textBounds = label.getLocalBounds();
    label.setOrigin({textBounds.position.x + textBounds.size.x / 2.0f,
                     textBounds.position.y + textBounds.size.y / 2.0f});
    label.setPosition({x + w / 2.0f, y + h / 2.0f});
}

void Button::render(sf::RenderWindow& window) {
    if (!isEnabled) {
        shape.setFillColor(disabledColor);
        label.setFillColor(sf::Color(100, 100, 100));
    } else if (isSelected) {
        shape.setFillColor(selectedColor);
        label.setFillColor(sf::Color::White);
    } else if (isHovered) {
        shape.setFillColor(hoverColor);
        label.setFillColor(sf::Color::White);
    } else {
        shape.setFillColor(baseColor);
        label.setFillColor(sf::Color::White);
    }
    
    window.draw(shape);
    window.draw(label);
}

void Button::update(float mx, float my) {
    isHovered = shape.getGlobalBounds().contains({mx, my});
}

bool Button::isClicked(float mx, float my) {
    return isEnabled && isHovered;
}

void Button::setEnabled(bool enabled) {
    isEnabled = enabled;
}

void Button::setSelected(bool selected) {
    isSelected = selected;
}

void Button::setText(const std::string& text) {
    label.setString(text);
    sf::FloatRect textBounds = label.getLocalBounds();
    label.setOrigin({textBounds.position.x + textBounds.size.x / 2.0f,
                     textBounds.position.y + textBounds.size.y / 2.0f});
    label.setPosition({x + width / 2.0f, y + height / 2.0f});
}
