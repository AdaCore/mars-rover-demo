//! Terrain model: elevation grid with traction derived from slope.

use noise::{NoiseFn, Perlin};

pub struct Terrain {
    elevations: Vec<f64>,
    grid_width: usize,
    grid_height: usize,
    x_min: f64,
    y_min: f64,
    cell_size: f64,
    t_min: f64,
    max_slope: f64,
}

impl Terrain {
    /// Create a new terrain using seeded Perlin noise.
    ///
    /// - `world_width` / `world_height`: extent of the terrain in world space,
    ///   centred at the origin.
    /// - `cell_size`: world-space distance between adjacent grid points.
    /// - `scale`: spatial frequency of the noise (larger = more features per metre).
    /// - `amplitude`: peak height above/below the nominal ground plane.
    /// - `t_min`: minimum traction fraction returned on the steepest slope.
    /// - `max_slope`: gradient magnitude that maps to `t_min`; shallower slopes
    ///   map linearly to 1.0.
    pub fn new(
        seed: u64,
        world_width: f64,
        world_height: f64,
        cell_size: f64,
        scale: f64,
        amplitude: f64,
        t_min: f64,
        max_slope: f64,
    ) -> Self {
        let grid_width = (world_width / cell_size).round() as usize + 1;
        let grid_height = (world_height / cell_size).round() as usize + 1;

        let x_min = -world_width / 2.0;
        let y_min = -world_height / 2.0;

        let perlin = Perlin::new(seed as u32);

        let mut elevations = Vec::with_capacity(grid_width * grid_height);
        for iy in 0..grid_height {
            for ix in 0..grid_width {
                let x = x_min + ix as f64 * cell_size;
                let y = y_min + iy as f64 * cell_size;
                elevations.push(amplitude * perlin.get([x * scale, y * scale]));
            }
        }

        Self {
            elevations,
            grid_width,
            grid_height,
            x_min,
            y_min,
            cell_size,
            t_min,
            max_slope,
        }
    }

    /// Height at an arbitrary world position via bilinear interpolation.
    pub fn height_at(&self, x: f64, y: f64) -> f64 {
        let gx = ((x - self.x_min) / self.cell_size).clamp(0.0, (self.grid_width - 1) as f64);
        let gy = ((y - self.y_min) / self.cell_size).clamp(0.0, (self.grid_height - 1) as f64);

        let ix = (gx.floor() as usize).min(self.grid_width - 2);
        let iy = (gy.floor() as usize).min(self.grid_height - 2);

        let tx = gx - ix as f64;
        let ty = gy - iy as f64;

        let h00 = self.elevations[iy * self.grid_width + ix];
        let h10 = self.elevations[iy * self.grid_width + ix + 1];
        let h01 = self.elevations[(iy + 1) * self.grid_width + ix];
        let h11 = self.elevations[(iy + 1) * self.grid_width + ix + 1];

        h00 * (1.0 - tx) * (1.0 - ty)
            + h10 * tx * (1.0 - ty)
            + h01 * (1.0 - tx) * ty
            + h11 * tx * ty
    }

    /// Gradient (∂h/∂x, ∂h/∂y) using central differences at interior points
    /// and forward/backward differences at the boundaries.
    pub fn gradient_at(&self, x: f64, y: f64) -> (f64, f64) {
        let eps = self.cell_size;
        let x_max = self.x_min + (self.grid_width - 1) as f64 * self.cell_size;
        let y_max = self.y_min + (self.grid_height - 1) as f64 * self.cell_size;

        let (x1, x2, dx) = if x <= self.x_min {
            (self.x_min, self.x_min + eps, eps)
        } else if x >= x_max {
            (x_max - eps, x_max, eps)
        } else {
            (x - eps, x + eps, 2.0 * eps)
        };

        let (y1, y2, dy) = if y <= self.y_min {
            (self.y_min, self.y_min + eps, eps)
        } else if y >= y_max {
            (y_max - eps, y_max, eps)
        } else {
            (y - eps, y + eps, 2.0 * eps)
        };

        (
            (self.height_at(x2, y) - self.height_at(x1, y)) / dx,
            (self.height_at(x, y2) - self.height_at(x, y1)) / dy,
        )
    }

    /// Traction factor in `[t_min, 1.0]`.  Flat ground (zero gradient) returns
    /// 1.0; gradient magnitude ≥ `max_slope` returns `t_min`.
    pub fn traction_at(&self, x: f64, y: f64) -> f64 {
        let (gx, gy) = self.gradient_at(x, y);
        let magnitude = (gx * gx + gy * gy).sqrt();
        1.0 - (1.0 - self.t_min) * (magnitude / self.max_slope).clamp(0.0, 1.0)
    }

    // Accessors used by the visualizer to build the terrain mesh.
    pub fn x_min(&self) -> f64 {
        self.x_min
    }
    pub fn y_min(&self) -> f64 {
        self.y_min
    }
    pub fn grid_width(&self) -> usize {
        self.grid_width
    }
    pub fn grid_height(&self) -> usize {
        self.grid_height
    }
    pub fn cell_size(&self) -> f64 {
        self.cell_size
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    /// Build a terrain from an explicit elevation array, bypassing Perlin noise.
    /// Useful for unit tests where the expected values must be precisely known.
    fn from_elevations(
        elevations: Vec<f64>,
        grid_width: usize,
        grid_height: usize,
        x_min: f64,
        y_min: f64,
        cell_size: f64,
        t_min: f64,
        max_slope: f64,
    ) -> Terrain {
        Terrain {
            elevations,
            grid_width,
            grid_height,
            x_min,
            y_min,
            cell_size,
            t_min,
            max_slope,
        }
    }

    #[test]
    fn test_height_at_grid_corners() {
        // 2×2 grid so all four vertices are "corners".
        let elevations = vec![1.0, 2.0, 3.0, 4.0];
        //  iy=0: ix=0 → 1.0,  ix=1 → 2.0
        //  iy=1: ix=0 → 3.0,  ix=1 → 4.0
        let t = from_elevations(elevations, 2, 2, 0.0, 0.0, 1.0, 0.3, 1.0);

        assert_abs_diff_eq!(t.height_at(0.0, 0.0), 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(t.height_at(1.0, 0.0), 2.0, epsilon = 1e-12);
        assert_abs_diff_eq!(t.height_at(0.0, 1.0), 3.0, epsilon = 1e-12);
        assert_abs_diff_eq!(t.height_at(1.0, 1.0), 4.0, epsilon = 1e-12);
    }

    #[test]
    fn test_height_at_cell_centre() {
        // The centre of the single cell (ix=0..1, iy=0..1) at world (0.5, 0.5)
        // should be the average of the four corner values.
        let elevations = vec![0.0, 2.0, 4.0, 6.0];
        let t = from_elevations(elevations, 2, 2, 0.0, 0.0, 1.0, 0.3, 1.0);
        let expected = (0.0 + 2.0 + 4.0 + 6.0) / 4.0;
        assert_abs_diff_eq!(t.height_at(0.5, 0.5), expected, epsilon = 1e-12);
    }

    #[test]
    fn test_traction_flat_terrain() {
        // amplitude = 0 → all elevations are 0 → gradient = 0 → traction = 1.0
        let t = Terrain::new(42, 2.0, 2.0, 0.1, 1.0, 0.0, 0.3, 0.5);
        assert_abs_diff_eq!(t.traction_at(0.0, 0.0), 1.0, epsilon = 1e-12);
        assert_abs_diff_eq!(t.traction_at(0.5, 0.3), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn test_traction_at_max_slope() {
        // Construct a linear ramp: h(x, y) = max_slope * x.
        // At any interior point, ∂h/∂x = max_slope, ∂h/∂y = 0,
        // so gradient magnitude = max_slope and traction must equal t_min.
        let max_slope = 0.5_f64;
        let t_min = 0.3_f64;
        let cell_size = 0.1_f64;
        let gw = 11_usize; // 0.0 to 1.0 in steps of 0.1
        let gh = 11_usize;
        let elevations: Vec<f64> = (0..gh)
            .flat_map(|_iy| (0..gw).map(move |ix| ix as f64 * cell_size * max_slope))
            .collect();
        let t = from_elevations(elevations, gw, gh, 0.0, 0.0, cell_size, t_min, max_slope);

        // Sample an interior point where central differences are available.
        assert_abs_diff_eq!(t.traction_at(0.5, 0.5), t_min, epsilon = 1e-9);
    }

    #[test]
    fn test_same_seed_produces_identical_terrain() {
        let make = || Terrain::new(99999, 2.0, 2.0, 0.1, 1.5, 0.02, 0.3, 0.05);
        let t1 = make();
        let t2 = make();
        assert_eq!(t1.elevations, t2.elevations);
    }
}
