#pragma once

#include <type_traits>
#include <utility>

namespace vamp::planning::configuration
{
    template <typename Robot, typename = void>
    struct Operations
    {
        using Configuration = typename Robot::Configuration;

        inline static auto difference(const Configuration &from, const Configuration &to) noexcept
            -> Configuration
        {
            return to - from;
        }

        inline static auto distance(const Configuration &from, const Configuration &to) noexcept -> float
        {
            return from.distance(to);
        }

        inline static auto
        interpolate(const Configuration &from, const Configuration &to, float alpha) noexcept -> Configuration
        {
            return from.interpolate(to, alpha);
        }
    };

    template <typename Robot>
    struct Operations<
        Robot,
        std::void_t<decltype(Robot::difference(
            std::declval<const typename Robot::Configuration &>(),
            std::declval<const typename Robot::Configuration &>()))>>
    {
        using Configuration = typename Robot::Configuration;

        inline static auto difference(const Configuration &from, const Configuration &to) noexcept
            -> Configuration
        {
            return Robot::difference(from, to);
        }

        inline static auto distance(const Configuration &from, const Configuration &to) noexcept -> float
        {
            return Robot::distance(from, to);
        }

        inline static auto
        interpolate(const Configuration &from, const Configuration &to, float alpha) noexcept -> Configuration
        {
            return Robot::interpolate(from, to, alpha);
        }
    };

    template <typename Robot>
    inline auto
    difference(const typename Robot::Configuration &from, const typename Robot::Configuration &to) noexcept ->
        typename Robot::Configuration
    {
        return Operations<Robot>::difference(from, to);
    }

    template <typename Robot>
    inline auto
    distance(const typename Robot::Configuration &from, const typename Robot::Configuration &to) noexcept
        -> float
    {
        return Operations<Robot>::distance(from, to);
    }

    template <typename Robot>
    inline auto interpolate(
        const typename Robot::Configuration &from,
        const typename Robot::Configuration &to,
        float alpha) noexcept -> typename Robot::Configuration
    {
        return Operations<Robot>::interpolate(from, to, alpha);
    }
}  // namespace vamp::planning::configuration
