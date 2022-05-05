#include <octomap/OcTree.h>

#include <array>
#include <cassert>
#include <iostream>

int main(int argc, char const* argv[]) {
    constexpr float default_resolution = 0.1f;
    const auto resolution = argc == 2 ? std::stof(argv[1]) : default_resolution;

    auto tree = octomap::OcTree(resolution);

    using dimensions = std::array<float, 3>;
    using position = std::array<float, 3>;
    using BBX = std::array<position, 2>;

    const auto create_perimeter = [&](const BBX bbx) {
        const auto [min, max] = bbx;
        const auto [x_min, y_min, z_min] = min;
        const auto [x_max, y_max, z_max] = max;
        assert(x_min < x_max);
        assert(y_min < y_max);
        assert(z_min < z_max);

        for (const auto x : {
                 x_min,
                 x_max,
             }) {
            for (float y = y_min; y < y_max; y += resolution) {
                for (float z = z_min; z < z_max; z += resolution) {
                    tree.updateNode(x, y, z, true);
                }
            }
        }

        for (const auto y : {
                 y_min,
                 y_max,
             }) {
            for (float x = x_min; x < x_max; x += resolution) {
                for (float z = z_min; z < z_max; z += resolution) {
                    tree.updateNode(x, y, z, true);
                }
            }
        }

        for (const auto z : {
                 z_min,
                 z_max,
             }) {
            for (float x = x_min; x < x_max; x += resolution) {
                for (float y = y_min; y < y_max; y += resolution) {
                    tree.updateNode(x, y, z, true);
                }
            }
        }
    };

    const auto create_bbx = [](const position& p, const dimensions& d) -> BBX {
        const auto [x_pos, y_pos, z_pos] = p;
        const auto [x_dim, y_dim, z_dim] = d;
        return {position{x_pos - x_dim / 2, y_pos - y_dim / 2, z_pos - z_dim / 2},
                position{x_pos + x_dim / 2, y_pos + y_dim / 2, z_pos + z_dim / 2}};
    };

    constexpr std::array<position, 3> thwomp_positions = {
        position{3.25, -1.137, 0.25 + 1}, position{4.0, 3.0, 3.0 + 1}, position{-3, 2.9, 1 + 1}};

    constexpr auto thwomp_dimensions = dimensions{2.0f, 2.0f, 2.0f};
    constexpr auto cage_dimensions = dimensions{14.0f, 14.0f, 6.f};
    constexpr auto cage_position = position{0.0f, 0.0f, 3.f};

    for (const auto pos : thwomp_positions) {
        create_perimeter(create_bbx(pos, thwomp_dimensions));
    }

    create_perimeter(create_bbx(cage_position, cage_dimensions));

    constexpr auto f = "airlab_cage.bt";

    tree.writeBinary(f);
    std::cerr << "writing octomap to " << f << '\n';

    return 0;
}
