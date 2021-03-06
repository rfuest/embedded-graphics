use crate::{
    draw_target::DrawTarget,
    drawable::{Drawable, Pixel},
    pixel_iterator::IntoPixels,
    pixelcolor::PixelColor,
    primitives::{polyline, polyline::Polyline, Primitive},
    style::{PrimitiveStyle, Styled},
};

/// Pixel iterator for each pixel in the line
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct StyledPixels<'a, C>
where
    C: PixelColor,
{
    stroke_color: Option<C>,
    line_iter: polyline::Points<'a>,
}

impl<'a, C> StyledPixels<'a, C>
where
    C: PixelColor,
{
    pub(in crate::primitives) fn new(styled: &Styled<Polyline<'a>, PrimitiveStyle<C>>) -> Self {
        StyledPixels {
            stroke_color: styled.style.effective_stroke_color(),
            line_iter: styled.primitive.points(),
        }
    }
}

impl<'a, C> Iterator for StyledPixels<'a, C>
where
    C: PixelColor,
{
    type Item = Pixel<C>;

    fn next(&mut self) -> Option<Self::Item> {
        // Return none if stroke color is none
        let stroke_color = self.stroke_color?;

        self.line_iter
            .next()
            .map(|point| Pixel(point, stroke_color))
    }
}

impl<'a, C> IntoPixels for &Styled<Polyline<'a>, PrimitiveStyle<C>>
where
    C: PixelColor,
{
    type Color = C;

    type Iter = StyledPixels<'a, C>;

    fn into_pixels(self) -> Self::Iter {
        StyledPixels::new(self)
    }
}

impl<'a, C> Drawable<C> for Styled<Polyline<'a>, PrimitiveStyle<C>>
where
    C: PixelColor,
{
    fn draw<D>(&self, display: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = C>,
    {
        display.draw_iter(self.into_pixels())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::polyline::tests::HEARTBEAT;
    use crate::primitives::polyline::tests::SMALL;
    use crate::{
        drawable::Drawable,
        geometry::Point,
        mock_display::MockDisplay,
        pixelcolor::{BinaryColor, Rgb565, RgbColor},
        primitives::Primitive,
        style::{PrimitiveStyle, PrimitiveStyleBuilder},
    };

    // Ensure that polylines only draw 1px wide due to lack of support for line joiners. This test
    // should fail when joiners are supported and should be removed then.
    #[test]
    fn one_px_wide_only() {
        let polyline = Polyline::new(&HEARTBEAT);

        let thick = polyline.into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 10));
        let thin = polyline.into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 1));

        assert!(thick.into_pixels().eq(thin.into_pixels()));
    }

    #[test]
    fn mock_display() {
        let mut display: MockDisplay<BinaryColor> = MockDisplay::new();

        Polyline::new(&SMALL)
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(&mut display)
            .unwrap();

        assert_eq!(
            display,
            MockDisplay::from_pattern(&[
                "                ",
                "                ",
                "     #         #",
                "    # ##     ## ",
                "   #    ## ##   ",
                "  #       #     ",
            ])
        );
    }

    #[test]
    fn empty_styled_iterators() {
        let points: [Point; 3] = [Point::new(2, 5), Point::new(3, 4), Point::new(4, 3)];

        // No stroke width = no pixels
        assert!(Polyline::new(&points)
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLUE, 0))
            .into_pixels()
            .eq(core::iter::empty()));

        // No stroke color = no pixels
        assert!(Polyline::new(&points)
            .into_styled::<Rgb565>(PrimitiveStyleBuilder::new().stroke_width(1).build())
            .into_pixels()
            .eq(core::iter::empty()));
    }
}
