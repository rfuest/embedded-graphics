//! Pixel iterator

use crate::drawable::Pixel;
use crate::{pixelcolor::PixelColor, DrawTarget};

/// Extension trait for pixel iterators.
pub trait PixelIteratorExt<C>
where
    C: PixelColor,
{
    /// Draws the pixel iterator to a draw target.
    fn draw<D>(self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = C>;
}

impl<I, C> PixelIteratorExt<C> for I
where
    C: PixelColor,
    I: Iterator<Item = Pixel<C>>,
{
    fn draw<D>(self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = C>,
    {
        target.draw_iter(self)
    }
}

/// Produce an iterator over all pixels in an object.
///
/// This trait is implemented for _references_ to all styled items in embedded-graphics, therefore
/// does not consume the original item.
pub trait IntoPixels {
    /// The type of color for each pixel produced by the iterator returned from [`into_pixels`].
    ///
    /// [`into_pixels`]: #tymethod.into_pixels
    type Color: PixelColor;

    /// The iterator produced when calling [`into_pixels`].
    ///
    /// [`into_pixels`]: #tymethod.into_pixels
    type Iter: Iterator<Item = Pixel<Self::Color>>;

    /// Create an iterator over all pixels in the object.
    ///
    /// The iterator may return pixels in any order, however it may be beneficial for performance
    /// reasons to return them starting at the top left corner in row-first order.
    fn into_pixels(self) -> Self::Iter;
}

// TODO: Implement as part of a new PR for sparse pixel iterators
// ///  TODO: Doc
// pub trait IntoSparsePixels<C>
// where
//     C: PixelColor,
// {
//     ///  TODO: Doc
//     type Iter: Iterator<Item = Option<C>> + Dimensions;

//     ///  TODO: Doc
//     fn into_sparse_pixels(self) -> Self::Iter;
// }
