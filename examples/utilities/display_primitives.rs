#[cfg(feature = "chrono")]
use chrono::{Datelike, NaiveDateTime, Timelike};
use embedded_graphics::image::Image;
use embedded_graphics::mono_font::{ascii, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Text;

#[allow(dead_code)]
pub fn colored_label<D>(
    text: &str,
    x: i32,
    y: i32,
    color: Rgb888,
    target: &mut D,
) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
{
    let style_red = PrimitiveStyleBuilder::new().fill_color(color).build();
    Rectangle::new(
        Point::new(x, y),
        Size::new((text.len() as u32) * 9 + 8, 22),
    )
        .into_styled(style_red)
        .draw(target)?;

    let text_style =
        MonoTextStyle::new(&ascii::FONT_9X18_BOLD, RgbColor::WHITE);
    Text::new(text, Point::new(x + 4, y + 18 - 2), text_style).draw(target)?;
    Ok(())
}

#[allow(dead_code)]
pub fn display_test<D>(
    target: &mut D,
) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
{
    let style_red =
        PrimitiveStyleBuilder::new().fill_color(Rgb888::RED).build();
    let style_green = PrimitiveStyleBuilder::new()
        .fill_color(Rgb888::GREEN)
        .build();
    let style_blue = PrimitiveStyleBuilder::new()
        .fill_color(Rgb888::BLUE)
        .build();
    let style_cyan = PrimitiveStyleBuilder::new()
        .fill_color(Rgb888::CYAN)
        .build();

    let size = target.bounding_box().size;
    Rectangle::new(Point::new(0, 0), Size::new(10, 10))
        .into_styled(style_red)
        .draw(target)?;
    Rectangle::new(Point::new(size.width as i32 - 10, 0), Size::new(10, 10))
        .into_styled(style_green)
        .draw(target)?;
    Rectangle::new(
        Point::new(0, size.height as i32 - 10),
        Size::new(10, 10),
    )
        .into_styled(style_blue)
        .draw(target)?;
    Rectangle::new(
        Point::new(size.width as i32 - 10, size.height as i32 - 10),
        Size::new(10, 10),
    )
        .into_styled(style_cyan)
        .draw(target)?;

    let ferris = tinybmp::Bmp::from_slice(include_bytes!("../ferris.bmp")).unwrap();
    let ferris = Image::new(&ferris, Point::new((size.width / 2 - ferris.size().width / 2) as i32, (size.height / 2 - ferris.size().height / 2) as i32));
    ferris.draw(target)?;
    Ok(())
}

#[cfg(feature = "chrono")]
#[allow(dead_code)]
pub fn seven_segment_style(color: Rgb888) -> eg_seven_segment::SevenSegmentStyle<Rgb888> {
    eg_seven_segment::SevenSegmentStyleBuilder::new()
        .digit_size(Size::new(36, 48)) // digits are 10x20 pixels
        .digit_spacing(5) // 5px spacing between digits
        .segment_width(5)
        .segment_color(color) // active segments are green
        .build()
}

#[cfg(feature = "chrono")]
#[allow(dead_code)]
pub fn date_labels<D>(y: i32, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
{
    let bg_color = Rgb888::RED;
    colored_label("MONTH", 86, y, bg_color, target)?;
    colored_label("DAY", 240, y, bg_color, target)?;
    colored_label("YEAR", 398, y, bg_color, target)?;
    colored_label("HOUR", 563, y, bg_color, target)?;
    colored_label("MIN", 692, y, bg_color, target)?;
    Ok(())
}

#[cfg(feature = "chrono")]
#[allow(dead_code)]
pub fn time_circuit<D>(dt: NaiveDateTime, x: i32, y: i32, label: &str, fg: Rgb888, bg: Rgb888, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Rgb888>,
{
    let fg_text_style = seven_segment_style(fg);
    let bg_text_style = seven_segment_style(bg);
    let all_segments = "888 88 8888 88 88";
    Text::new(all_segments, Point::new(x, y), bg_text_style)
        .draw(target)?;
    let month = match dt.date().month() {
        1 => "JAN",
        2 => "FEB",
        3 => "MAR",
        4 => "APR",
        5 => "MAY",
        6 => "JUN",
        7 => "JUL",
        8 => "AUG",
        9 => "SEP",
        10 => "OCT",
        11 => "NOU",
        12 => "DEC",
        _ => unreachable!()
    };
    let mut buf = [0u8; 17];
    use crate::utilities::write::write_to::WriteTo;
    use core::fmt::Write;
    let mut buf = WriteTo::new(&mut buf);
    let (is_pm, hour) = dt.time().hour12();
    write!(&mut buf, "{} {:02} {:04} {:02} {:02}", month, dt.date().day(), dt.date().year(), hour, dt.time().minute()).unwrap();
    Text::new(buf.as_str().unwrap(), Point::new(x, y), fg_text_style)
        .draw(target)?;
    date_labels(y - 75, target)?;
    colored_label(
        label,
        x + 363 - ((label.len() as i32) / 2) * 9,
        y + 10,
        Rgb888::CSS_DIM_GRAY,
        target,
    )?;

    let fg_style = PrimitiveStyleBuilder::new()
        .fill_color(fg)
        .build();
    let bg_style = PrimitiveStyleBuilder::new()
        .fill_color(bg)
        .build();
    colored_label("AM", 633, y - 65, Rgb888::RED, target)?;
    let (am_style, pm_style) = if is_pm {
        (bg_style, fg_style)
    } else {
        (fg_style, bg_style)
    };
    use embedded_graphics::primitives::Circle;
    Circle::new(Point::new(641, y - 40), 10).into_styled(am_style).draw(target)?;
    colored_label("PM", 633, y - 26, Rgb888::RED, target)?;
    Circle::new(Point::new(641, y), 10).into_styled(pm_style).draw(target)?;
    Ok(())
}
