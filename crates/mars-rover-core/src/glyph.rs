//! Stroke-font glyph table and text-to-waypoints conversion.
//!
//! ## Coordinate system
//!
//! Each glyph is defined in integer font units:
//!
//! * x ∈ [0, `advance`] — left edge to right bearing
//! * y = 0              — baseline
//! * y = 6              — cap height / ascender
//! * y = 4              — lowercase x-height
//! * y = -2             — descender
//!
//! ## Pen-up
//!
//! `PEN_UP` marks the end of one stroke and the start of the next within a
//! glyph.  When converting to waypoints the sentinel is skipped; the rover
//! drives directly from the last point of one stroke to the first point of
//! the next (the transit move is a normal waypoint path segment).

const PEN_UP: (i8, i8) = (i8::MIN, i8::MIN);
type GlyphDef = (i8, &'static [(i8, i8)]);

/// Glyph table for ASCII 32 (` `) … 126 (`~`), one entry per code point.
#[rustfmt::skip]
static GLYPHS: &[GlyphDef] = &[
    // 32 ' '
    (3, &[]),
    // 33 '!'
    (2, &[(1,1),(1,2), PEN_UP, (1,4),(1,6)]),
    // 34 '"'
    (5, &[(1,5),(1,6), PEN_UP, (3,5),(3,6)]),
    // 35 '#'
    (6, &[(2,0),(2,6), PEN_UP, (4,0),(4,6), PEN_UP, (0,4),(6,4), PEN_UP, (0,2),(6,2)]),
    // 36 '$'
    (6, &[(3,6),(3,0), PEN_UP, (5,5),(4,6),(2,6),(1,5),(2,4),(4,4),(5,3),(4,2),(2,2),(1,1),(2,0),(4,0),(5,1)]),
    // 37 '%'
    (6, &[(1,0),(5,6), PEN_UP, (1,5),(2,6),(2,5),(1,6),(1,5), PEN_UP, (4,1),(5,0),(5,1),(4,0),(4,1)]),
    // 38 '&'
    (7, &[(6,0),(4,0),(2,1),(2,3),(5,5),(4,6),(2,5),(1,3),(2,1),(5,4),(6,3)]),
    // 39 '\''
    (2, &[(1,5),(1,6)]),
    // 40 '('
    (4, &[(3,6),(2,4),(2,2),(3,0)]),
    // 41 ')'
    (4, &[(1,6),(2,4),(2,2),(1,0)]),
    // 42 '*'
    (5, &[(1,3),(5,3), PEN_UP, (2,5),(4,1), PEN_UP, (4,5),(2,1)]),
    // 43 '+'
    (6, &[(0,3),(6,3), PEN_UP, (3,0),(3,6)]),
    // 44 ','
    (2, &[(1,1),(1,-1)]),
    // 45 '-'
    (5, &[(0,3),(5,3)]),
    // 46 '.'
    (2, &[(1,0),(1,1)]),
    // 47 '/'
    (5, &[(0,0),(5,6)]),
    // 48 '0'
    (6, &[(5,1),(4,0),(2,0),(1,1),(0,3),(1,5),(2,6),(4,6),(5,5),(5,1)]),
    // 49 '1'
    (4, &[(1,5),(2,6),(2,0)]),
    // 50 '2'
    (6, &[(1,5),(2,6),(4,6),(5,5),(5,4),(1,0),(6,0)]),
    // 51 '3'
    (6, &[(1,6),(5,6),(5,4),(3,3),(5,2),(5,1),(3,0),(1,0)]),
    // 52 '4'
    (6, &[(4,0),(4,6),(0,2),(6,2)]),
    // 53 '5'
    (6, &[(5,6),(1,6),(1,4),(3,4),(5,3),(5,1),(3,0),(1,1)]),
    // 54 '6'
    (6, &[(5,6),(3,6),(1,5),(1,1),(2,0),(4,0),(5,1),(5,3),(3,4),(1,3)]),
    // 55 '7'
    (6, &[(0,6),(6,6),(3,0)]),
    // 56 '8'
    (6, &[(3,3),(1,2),(1,1),(2,0),(4,0),(5,1),(5,2),(3,3),(5,4),(5,5),(4,6),(2,6),(1,5),(1,4),(3,3)]),
    // 57 '9'
    (6, &[(5,3),(5,5),(4,6),(2,6),(1,5),(1,3),(2,2),(4,2),(5,3),(5,0)]),
    // 58 ':'
    (2, &[(1,0),(1,1), PEN_UP, (1,3),(1,4)]),
    // 59 ';'
    (2, &[(1,-1),(1,1), PEN_UP, (1,3),(1,4)]),
    // 60 '<'
    (5, &[(4,5),(1,3),(4,1)]),
    // 61 '='
    (6, &[(0,2),(6,2), PEN_UP, (0,4),(6,4)]),
    // 62 '>'
    (5, &[(1,5),(4,3),(1,1)]),
    // 63 '?'
    (5, &[(1,5),(2,6),(4,6),(5,5),(5,4),(3,3),(3,2), PEN_UP, (3,0),(3,1)]),
    // 64 '@'  — simplified to a circle
    (7, &[(6,3),(5,5),(3,6),(1,5),(0,3),(1,1),(3,0),(5,1),(6,3)]),
    // 65 'A'
    (6, &[(0,0),(3,6),(6,0), PEN_UP, (1,2),(5,2)]),
    // 66 'B'
    (6, &[(1,0),(1,6),(3,6),(5,5),(5,4),(3,3),(1,3),(3,3),(5,2),(5,1),(3,0),(1,0)]),
    // 67 'C'
    (6, &[(5,5),(4,6),(2,6),(1,5),(0,3),(1,1),(2,0),(4,0),(5,1)]),
    // 68 'D'
    (6, &[(1,0),(1,6),(3,6),(5,5),(6,3),(5,1),(3,0),(1,0)]),
    // 69 'E'
    (5, &[(5,6),(1,6),(1,0),(5,0), PEN_UP, (1,3),(4,3)]),
    // 70 'F'
    (5, &[(5,6),(1,6),(1,0), PEN_UP, (1,3),(4,3)]),
    // 71 'G'
    (7, &[(6,5),(4,6),(2,6),(1,5),(0,3),(1,1),(2,0),(4,0),(6,1),(6,3),(4,3)]),
    // 72 'H'
    (6, &[(1,0),(1,6), PEN_UP, (5,0),(5,6), PEN_UP, (1,3),(5,3)]),
    // 73 'I'
    (4, &[(0,6),(4,6), PEN_UP, (2,6),(2,0), PEN_UP, (0,0),(4,0)]),
    // 74 'J'
    (5, &[(5,6),(5,1),(4,0),(2,0),(1,1)]),
    // 75 'K'
    (6, &[(1,0),(1,6), PEN_UP, (5,6),(1,3),(5,0)]),
    // 76 'L'
    (5, &[(1,6),(1,0),(5,0)]),
    // 77 'M'
    (7, &[(1,0),(1,6),(4,3),(7,6),(7,0)]),
    // 78 'N'
    (6, &[(1,0),(1,6),(5,0),(5,6)]),
    // 79 'O'
    (6, &[(5,1),(4,0),(2,0),(1,1),(0,3),(1,5),(2,6),(4,6),(5,5),(5,1)]),
    // 80 'P'
    (6, &[(1,0),(1,6),(3,6),(5,5),(5,4),(3,3),(1,3)]),
    // 81 'Q'
    (6, &[(5,1),(4,0),(2,0),(1,1),(0,3),(1,5),(2,6),(4,6),(5,5),(5,1), PEN_UP, (4,1),(6,0)]),
    // 82 'R'
    (6, &[(1,0),(1,6),(3,6),(5,5),(5,4),(3,3),(1,3),(5,0)]),
    // 83 'S'
    (6, &[(5,1),(4,0),(2,0),(1,1),(1,2),(2,3),(4,3),(5,4),(5,5),(4,6),(2,6),(1,5)]),
    // 84 'T'
    (6, &[(0,6),(6,6), PEN_UP, (3,6),(3,0)]),
    // 85 'U'
    (6, &[(1,6),(1,1),(2,0),(4,0),(5,1),(5,6)]),
    // 86 'V'
    (6, &[(0,6),(3,0),(6,6)]),
    // 87 'W'
    (8, &[(0,6),(2,0),(4,3),(6,0),(8,6)]),
    // 88 'X'
    (6, &[(0,6),(6,0), PEN_UP, (0,0),(6,6)]),
    // 89 'Y'
    (6, &[(0,6),(3,3),(6,6), PEN_UP, (3,3),(3,0)]),
    // 90 'Z'
    (6, &[(0,6),(6,6),(0,0),(6,0)]),
    // 91 '['
    (3, &[(2,0),(1,0),(1,6),(2,6)]),
    // 92 '\'
    (5, &[(0,6),(5,0)]),
    // 93 ']'
    (3, &[(1,0),(2,0),(2,6),(1,6)]),
    // 94 '^'
    (5, &[(1,4),(3,6),(5,4)]),
    // 95 '_'
    (6, &[(0,0),(6,0)]),
    // 96 '`'
    (3, &[(2,6),(1,5)]),
    // 97 'a'
    (6, &[(5,4),(4,4),(2,4),(1,3),(1,1),(2,0),(4,0),(5,1),(5,0)]),
    // 98 'b'
    (6, &[(1,6),(1,0), PEN_UP, (1,3),(2,4),(4,4),(5,3),(5,1),(4,0),(2,0),(1,1)]),
    // 99 'c'
    (6, &[(5,3),(4,4),(2,4),(1,3),(1,1),(2,0),(4,0),(5,1)]),
    // 100 'd'
    (6, &[(5,0),(5,6), PEN_UP, (5,4),(4,4),(2,4),(1,3),(1,1),(2,0),(4,0),(5,1)]),
    // 101 'e'
    (6, &[(1,2),(5,2),(5,3),(4,4),(2,4),(1,3),(1,1),(2,0),(4,0),(5,1)]),
    // 102 'f'
    (5, &[(4,6),(3,6),(2,5),(2,0), PEN_UP, (0,3),(4,3)]),
    // 103 'g'
    (6, &[(5,4),(4,4),(2,4),(1,3),(1,1),(2,0),(4,0),(5,1),(5,-1),(4,-2),(2,-2)]),
    // 104 'h'
    (6, &[(1,6),(1,0), PEN_UP, (1,3),(2,4),(4,4),(5,3),(5,0)]),
    // 105 'i'
    (3, &[(2,5),(2,6), PEN_UP, (2,0),(2,4)]),
    // 106 'j'
    (4, &[(3,5),(3,6), PEN_UP, (3,4),(3,-1),(2,-2)]),
    // 107 'k'
    (6, &[(1,0),(1,6), PEN_UP, (5,4),(1,2),(5,0)]),
    // 108 'l'
    (3, &[(2,0),(2,6)]),
    // 109 'm'
    (8, &[(1,0),(1,4),(2,4),(3,3),(3,0), PEN_UP, (3,3),(4,4),(6,4),(7,3),(7,0)]),
    // 110 'n'
    (6, &[(1,0),(1,4),(2,4),(4,4),(5,3),(5,0)]),
    // 111 'o'
    (6, &[(5,3),(4,4),(2,4),(1,3),(1,1),(2,0),(4,0),(5,1),(5,3)]),
    // 112 'p'
    (6, &[(1,-2),(1,3),(2,4),(4,4),(5,3),(5,1),(4,0),(2,0),(1,1)]),
    // 113 'q'
    (6, &[(5,3),(4,4),(2,4),(1,3),(1,1),(2,0),(4,0),(5,1),(5,-2)]),
    // 114 'r'
    (5, &[(1,0),(1,4),(2,4),(4,4)]),
    // 115 's'
    (5, &[(5,4),(4,4),(2,4),(1,3),(2,2),(4,2),(5,1),(4,0),(2,0),(1,1)]),
    // 116 't'
    (5, &[(2,6),(2,0), PEN_UP, (0,3),(5,3)]),
    // 117 'u'
    (6, &[(1,4),(1,1),(2,0),(4,0),(5,1),(5,4)]),
    // 118 'v'
    (6, &[(0,4),(3,0),(6,4)]),
    // 119 'w'
    (8, &[(0,4),(2,0),(4,2),(6,0),(8,4)]),
    // 120 'x'
    (6, &[(0,4),(6,0), PEN_UP, (0,0),(6,4)]),
    // 121 'y'
    (6, &[(0,4),(3,1),(6,4), PEN_UP, (3,1),(1,-2)]),
    // 122 'z'
    (5, &[(0,4),(5,4),(0,0),(5,0)]),
    // 123 '{'
    (4, &[(3,6),(2,5),(2,4),(1,3),(2,2),(2,1),(3,0)]),
    // 124 '|'
    (2, &[(1,0),(1,6)]),
    // 125 '}'
    (4, &[(1,6),(2,5),(2,4),(3,3),(2,2),(2,1),(1,0)]),
    // 126 '~'
    (6, &[(0,3),(1,4),(3,2),(5,3),(6,4)]),
];

fn glyph_for(c: char) -> GlyphDef {
    let code = c as usize;
    if (32..=126).contains(&code) {
        GLYPHS[code - 32]
    } else {
        GLYPHS[0] // fall back to space
    }
}

/// Convert a text string to a flat list of `(x, y)` world-space waypoints,
/// centred in a world of the given dimensions.
///
/// The text block is scaled to fill at most [`H_FILL`] × `world_width`
/// horizontally and [`V_FILL`] × `world_height` vertically, whichever is
/// the tighter constraint.  This means the function works correctly regardless
/// of world size — make the world bigger and the letters grow to match.
///
/// [`PEN_UP`] moves between strokes are included as ordinary waypoints; the
/// rover drives directly from the end of one stroke to the start of the next.
pub fn text_to_waypoints(text: &str, world_width: f64, world_height: f64) -> Vec<(f64, f64)> {
    /// Fraction of world width used for the text block.
    const H_FILL: f64 = 0.85;
    /// Fraction of world height used for the text block.
    const V_FILL: f64 = 0.70;
    /// Cap height in font units (top of uppercase letters).
    const FONT_CAP: f64 = 6.0;
    /// Descender depth below baseline in font units.
    const FONT_DESC: f64 = 2.0;
    /// Total font height (ascender top to descender bottom).
    const FONT_FULL: f64 = FONT_CAP + FONT_DESC;
    /// Extra horizontal gap inserted between adjacent characters (font units).
    const INTER_CHAR: f64 = 1.0;

    let chars: Vec<char> = text.chars().collect();
    if chars.is_empty() {
        return vec![];
    }

    // Total advance width of the text block (no trailing inter-char gap).
    let total_advance: f64 = chars.iter().map(|&c| glyph_for(c).0 as f64).sum::<f64>()
        + (chars.len() - 1) as f64 * INTER_CHAR;

    if total_advance <= 0.0 {
        return vec![];
    }

    // Scale: constrained by both horizontal and vertical fill limits.
    let scale = (world_width * H_FILL / total_advance).min(world_height * V_FILL / FONT_FULL);

    // Baseline y in world coordinates (text block is vertically centred).
    let y_baseline = (FONT_DESC - FONT_FULL * 0.5) * scale;

    // Left edge of first glyph in world coordinates.
    let x_origin = -total_advance * scale * 0.5;

    let mut waypoints = Vec::new();
    let mut cursor_x = x_origin;

    for &c in &chars {
        let (advance, strokes) = glyph_for(c);
        for &(fx, fy) in strokes {
            if (fx, fy) != PEN_UP {
                waypoints.push((
                    cursor_x + fx as f64 * scale,
                    y_baseline + fy as f64 * scale,
                ));
            }
        }
        cursor_x += (advance as f64 + INTER_CHAR) * scale;
    }

    waypoints
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn glyph_table_has_correct_length() {
        assert_eq!(GLYPHS.len(), 95, "expected one entry per ASCII 32–126");
    }

    #[test]
    fn adacore_produces_waypoints() {
        let wps = text_to_waypoints("AdaCore", 5.0, 3.0);
        assert!(!wps.is_empty(), "should produce waypoints for 'AdaCore'");
    }

    #[test]
    fn empty_string_produces_no_waypoints() {
        let wps = text_to_waypoints("", 5.0, 3.0);
        assert!(wps.is_empty());
    }

    #[test]
    fn waypoints_fit_within_world() {
        let (ww, wh) = (5.0_f64, 3.0_f64);
        let wps = text_to_waypoints("AdaCore", ww, wh);
        for (x, y) in &wps {
            assert!(
                x.abs() <= ww / 2.0,
                "x={x} out of world width {ww}"
            );
            assert!(
                y.abs() <= wh / 2.0,
                "y={y} out of world height {wh}"
            );
        }
    }

    #[test]
    fn waypoints_scale_with_world_size() {
        let small = text_to_waypoints("A", 5.0, 3.0);
        let large = text_to_waypoints("A", 20.0, 12.0);
        // Larger world → larger letters → greater spread from origin.
        let max_small = small.iter().map(|(x, _)| x.abs()).fold(0.0_f64, f64::max);
        let max_large = large.iter().map(|(x, _)| x.abs()).fold(0.0_f64, f64::max);
        assert!(max_large > max_small, "larger world should produce larger letters");
    }
}
