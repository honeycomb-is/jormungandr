#pragma once

#include <cstdint>
#include "../ecs/Components.hpp"
#include "../ecs/ElementComponents.hpp"

namespace Engine::Species
{

    enum class EntityType : std::uint8_t
    {
        Atom,
        Electron,
        Ion
    };

    struct Atom
    {
        EntityType type{EntityType::Atom};
        // Visuals
        Engine::ECS::Radius visualRadius{1.0f};
        int charge_e{0}; // net charge in elementary charges
        // Element identity and optional properties
        Engine::ECS::ElementComponentSet element;
    };

    struct Electron
    {
        EntityType type{EntityType::Electron};
        Engine::ECS::Radius visualRadius{0.2f};
        int charge_e{-1};
    };

    struct Ion
    {
        EntityType type{EntityType::Ion};
        int charge_e{+1};
        Engine::ECS::Radius visualRadius{0.9f};
    };

} // namespace Engine::Species
