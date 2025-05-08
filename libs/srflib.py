import rhinoscriptsyntax as rs
from System.Drawing import Bitmap
import geometrylib as gl

def closest_srf(pt, srf0, srf1):
    """returns the surface that is closest to a certain point"""
    param0 = rs.SurfaceClosestPoint(srf0, pt)
    param1 = rs.SurfaceClosestPoint(srf1, pt)
    srf0_pt = rs.EvaluateSurface(srf0, param0[0] , param0[1])
    srf1_pt = rs.EvaluateSurface(srf1, param1[0] , param1[1])
    d0 = rs.Distance(pt, srf0_pt)
    d1 = rs.Distance(pt, srf1_pt)
    print('distance: ', d0, d1)
    if d0 >= d1:
        return(srf1, 1)
    else:
        return(srf0, 0)

def sample_surface_color(pt, surface, image_path, as_hsl=False):
    """
    Sample the color of a texture mapped to a Rhino surface at the closest point to a given 3D point.

    Parameters:
        pt (Point3d or list/tuple of 3 floats): The 3D point to sample from.
        surface (GUID): The Rhino surface or polysurface to sample.
        image_path (str): File path to the texture image.
        as_hsl (bool): If True, returns the color as (H, S, L); otherwise as (R, G, B).

    Returns:
        tuple: (R, G, B) or (H, S, L) values of the sampled texture at the closest surface point.
    """
    # Ensure we have a Point3d
    pt3d = rs.coerce3dpoint(pt)

    # Load the image
    img = Bitmap(image_path)

    # Find UV parameter on surface for closest point
    u, v = rs.SurfaceClosestPoint(surface, pt3d)

    # Surface domains in U and V
    u_dom = rs.SurfaceDomain(surface, 0)
    v_dom = rs.SurfaceDomain(surface, 1)

    # Image dimensions
    img_w = img.Width
    img_h = img.Height

    # Map U to pixel X and V to pixel Y (flip V because image origin is top-left)
    px = int(round(gl.remap(u_dom[0], u_dom[1], 0, img_w - 1, u)))
    py = int(round(gl.remap(v_dom[0], v_dom[1], img_h - 1, 0, v)))

    # Sample pixel color
    color = img.GetPixel(px, py)

    if as_hsl:
        # Rhino's ColorRGBToHLS returns (H, L, S)
        h, l, s = rs.ColorRGBToHLS(color)
        # Return as (H, S, L)
        return h, s, l
    else:
        # Return (R, G, B)
        return color.R, color.G, color.B

def remap_rgb_channels(rgb, *channel_ranges):
    """
    General remapping of RGB channels to value ranges.

    Parameters:
        rgb (tuple): RGB tuple (R, G, B), each 0–255.
        *channel_ranges: Three tuples, each (min, max), corresponding to
                         remap targets for R, G, B channels.

    Returns:
        tuple: Remapped float values, one per channel.

    Example:
        >>> rgb_to_parameters((128, 64, 255), (2, 5), (0.5, 1.5), (0, 2))
        (3.5, 0.75, 2.0)
    """

    r, g, b = rgb
    param1 = gl.remap(0, 255, *range_1, r)
    param2 = gl.remap(0, 255, *range_2, g)
    param3 = gl.remap(0, 255, *range_3, b)
    return param1, param2, param3
