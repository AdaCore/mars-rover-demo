//! Test utils.

use textplots::{Chart, Plot, Shape};

macro_rules! set_snapshot_suffix {
    ($($expr:expr),*) => {
        let mut settings = insta::Settings::clone_current();
        settings.set_snapshot_suffix(format!($($expr,)*));
        let _guard = settings.bind_to_scope();
    }
}

pub fn plot_point_chart(points: &[(f32, f32)]) -> String {
    let mut chart = Chart::new_with_y_range(100, 100, -1.0, 1.0, -1.0, 1.0);
    let binding = Shape::Points(points);
    let chart = chart.lineplot(&binding);
    chart.figures();
    chart.to_string()
}

pub fn plot_line_chart(points: &[Vec<(f32, f32)>]) -> String {
    let mut chart = Chart::new_with_y_range(100, 100, -1.0, 1.0, -1.0, 1.0);
    let chart = chart.lineplot(&Shape::Lines(&[]));
    let lines = points.iter().map(|p| Shape::Lines(p)).collect::<Vec<_>>();
    let chart = lines.iter().fold(chart, |c, l| c.lineplot(l));
    chart.figures();
    chart.to_string()
}
