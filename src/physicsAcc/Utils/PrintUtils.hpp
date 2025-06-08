#pragma once

// STD
#include <iostream>
#include <iomanip>
#include <string>
#include <tuple>
#include <utility> 
#include <type_traits>

// Internal
#include "physicsAcc/Body/BodyCollection.hpp"
#include "physicsAcc/Constraints/ConstraintCollection.hpp"
#include "physicsAcc/Constraints/Joint.hpp"

template <typename T>
struct type_identity {};

// Descriptor para BodyCollection
inline auto describe_collection(type_identity<BodyCollection>) {
    return std::make_tuple(
        std::make_pair("Type", &BodyCollection::type),
        std::make_pair("Mass", &BodyCollection::mass),
        std::make_pair("Inverse Mass", &BodyCollection::inverse_mass),
        std::make_pair("Position", &BodyCollection::position),
        std::make_pair("Orientation", &BodyCollection::orientation),
        std::make_pair("Linear Velocity", &BodyCollection::linear_velocity),
        std::make_pair("Angular Velocity", &BodyCollection::angular_velocity),
        std::make_pair("Force", &BodyCollection::force),
        std::make_pair("Torque", &BodyCollection::torque),
        std::make_pair("Inertia Tensor", &BodyCollection::inertia_tensor),
        std::make_pair("Inverse Inertia Tensor", &BodyCollection::inverse_inertia_tensor),
        std::make_pair("Inertia Tensor World", &BodyCollection::inertia_tensor_world),
        std::make_pair("Inverse Inertia Tensor World", &BodyCollection::inverse_inertia_tensor_world)
    );
}

// Descriptor para ConstraintCollection
inline auto describe_collection(type_identity<ConstraintCollection>) {
    return std::make_tuple(
        std::make_pair("Type", &ConstraintCollection::type),
        std::make_pair("Body 1 Index", &ConstraintCollection::body_1),
        std::make_pair("Body 2 Index", &ConstraintCollection::body_2),
        std::make_pair("Direction", &ConstraintCollection::direction),
        std::make_pair("Compliance", &ConstraintCollection::compliance),
        std::make_pair("Lambda (Lagrange)", &ConstraintCollection::lambda),
        std::make_pair("Force", &ConstraintCollection::force),
        std::make_pair("Torque", &ConstraintCollection::torque),
        std::make_pair("Local Point r1", &ConstraintCollection::r_1),
        std::make_pair("Local Point r2", &ConstraintCollection::r_2),
        std::make_pair("Magnitude", &ConstraintCollection::magnitude), 
        std::make_pair("Impulse", &ConstraintCollection::impulse) 
    );
}

// Descriptor para JointCollection (¡Nuevo!)
inline auto describe_collection(type_identity<JointCollection>) {
    return std::make_tuple(
        std::make_pair("Type", &JointCollection::type),
        std::make_pair("Actuation Type", &JointCollection::actuation_type),
        std::make_pair("Body 1 Index", &JointCollection::body_1),
        std::make_pair("Body 2 Index", &JointCollection::body_2),
        std::make_pair("Local Point r1", &JointCollection::r_1),
        std::make_pair("Local Point r2", &JointCollection::r_2),
        std::make_pair("Main Axis", &JointCollection::main_axis),
        std::make_pair("Limited", &JointCollection::limited),
        std::make_pair("Lower Limit", &JointCollection::lower_limit),
        std::make_pair("Upper Limit", &JointCollection::upper_limit),
        std::make_pair("Target Position", &JointCollection::target_position),
        std::make_pair("Target Speed", &JointCollection::target_speed),
        std::make_pair("Current Position", &JointCollection::current_position),
        std::make_pair("Damping", &JointCollection::damping),
        std::make_pair("Limit Axis", &JointCollection::limit_axis),
        std::make_pair("Constraint Start", &JointCollection::constraint_start),
        std::make_pair("Constraint Count", &JointCollection::constraint_count)
    );
}

// =================================================================
// PASO 2 Y 3: EL MOTOR GENÉRICO DE IMPRESIÓN
// Este código es completamente genérico y no necesita ser modificado.
// =================================================================

// Ayudas para imprimir enums de forma legible
std::ostream& operator<<(std::ostream& os, const BodyType& type) {
    os << (type == STATIC ? "STATIC" : "DYNAMIC");
    return os;
}

std::ostream& operator<<(std::ostream& os, const ConstraintType& type) {
    os << (type == POSITIONAL ? "POSITIONAL" : "ROTATIONAL");
    return os;
}

// ¡Nuevo! Sobrecargas para JointType y JointActuationType
std::ostream& operator<<(std::ostream& os, const JointType& type) {
    switch (type) {
        case PRISMATIC: os << "PRISMATIC"; break;
        case REVOLUTE: os << "REVOLUTE"; break;
        case FIXED: os << "FIXED"; break;
        default: os << "UNKNOWN_JOINT_TYPE"; break;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const JointActuationType& type) {
    switch (type) {
        case FREE: os << "FREE"; break;
        case POSITION: os << "POSITION"; break;
        case SPEED: os << "SPEED"; break;
        default: os << "UNKNOWN_ACTUATION_TYPE"; break;
    }
    return os;
}

// Estructura de información genérica que contiene los datos y su descripción
template <typename LayoutTuple, typename ValueTuple>
struct GenericDebugInfo {
    const char* collection_name;
    size_t index;
    LayoutTuple layout;
    ValueTuple values;

    // Constructor explícito para GenericDebugInfo (corregido de la vez anterior)
    GenericDebugInfo(const char* name, size_t idx, LayoutTuple l, ValueTuple v)
        : collection_name(name), index(idx), layout(std::move(l)), values(std::move(v)) {}
};

// Generador de información genérico
template <typename TCollection>
auto get_generic_info(const TCollection& collection, size_t index, const char* name) {
    auto layout = describe_collection(type_identity<TCollection>{});

    auto values = std::apply(
        [&](const auto&... pairs) {
            return std::make_tuple((collection.*(pairs.second))[index]...);
        },
        layout
    );

    return GenericDebugInfo{name, index, layout, values};
}

template <typename T>
struct is_math_type : std::false_type {};

template <> struct is_math_type<vec2> : std::true_type {};
template <> struct is_math_type<vec3> : std::true_type {};
template <> struct is_math_type<quat> : std::true_type {};
template <> struct is_math_type<mat3> : std::true_type {};
template <> struct is_math_type<mat4> : std::true_type {};

// Ayuda interna para imprimir una tupla de manera recursiva en tiempo de compilación
template<size_t I = 0, typename LayoutTuple, typename ValueTuple>
void print_info_elements(std::ostream& os, const LayoutTuple& layout, const ValueTuple& values) {
    if constexpr (I < std::tuple_size_v<LayoutTuple>) {
        const int label_width = 24; // Ancho fijo para la etiqueta
        const int indent_after_label = label_width + 4; // 4 para "  - "

        const auto& pair = std::get<I>(layout);
        const auto& value = std::get<I>(values);

        // Imprime el prefijo y la etiqueta alineada
        os << "  - " << std::left << std::setw(label_width) << std::string(pair.first) + ": ";

        // Si es una matriz, imprime un salto de línea antes del valor
        // y asegúrate de que el operator<< de la matriz use la indentación correcta.
        // Si no es una matriz, o es un tipo simple, imprímelo en la misma línea.
        if constexpr (std::is_same_v<std::decay_t<decltype(value)>, mat3> ||
                      std::is_same_v<std::decay_t<decltype(value)>, mat4>) {
            // Para matrices, queremos el valor en la siguiente línea con indentación
            os << "\n";
            // Aquí no necesitamos una indentación adicional porque el ToString de la matriz
            // se encargará de ello al imprimir su contenido.
            // Solo aseguramos que el flujo continúe en una nueva línea.
        } else {
            // Para otros tipos (vec, quat, escalar, enum), imprimir directamente
            // No añadimos un salto de línea extra aquí. El salto de línea final lo hará.
        }

        // Imprime el valor. Los operator<< sobrecargados se encargan del formato interno.
        os << value;
        os << "\n"; // Un salto de línea después de cada valor para consistencia

        print_info_elements<I + 1>(os, layout, values);
    }
}

// Impresor genérico para nuestra estructura de información
template <typename LayoutTuple, typename ValueTuple>
std::ostream& operator<<(std::ostream& os, const GenericDebugInfo<LayoutTuple, ValueTuple>& info) {
    std::string title = std::string("======== ") + info.collection_name + " [Index: " + std::to_string(info.index) + "] ========";
    std::string line(title.length(), '=');

    os << std::fixed << std::setprecision(4);
    os << line << "\n";
    os << title << "\n";
    os << line << "\n";

    print_info_elements(os, info.layout, info.values);

    os << line << "\n";
    return os;
}