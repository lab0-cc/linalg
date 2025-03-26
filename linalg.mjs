// This module provides the linear algebra primitives required by 0WM. It is in no way intended as
// a general-purpose library.


// 2D vector class
export class Vector2 {
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }

    // Return a copy of the vector
    copy() {
        return new this.constructor(this.x, this.y);
    }

    // Return a new vector shifted by a vector
    plus(v) {
        return new this.constructor(this.x + v.x, this.y + v.y);
    }

    // Return a new vector shifted by the opposite of a vector
    minus(v) {
        return new this.constructor(this.x - v.x, this.y - v.y);
    }

    // Return the 2D cross product (or 2×2 matrix determinant)
    cross(v) {
        return this.x * v.y - this.y * v.x;
    }

    // Return a new vector rotated by the given angle
    rotated(angle) {
        return new this.constructor(this.x * angle.cos - this.y * angle.sin,
                                    this.x * angle.sin + this.y * angle.cos);
    }

    // Return the angle relative to the x unit vector
    angle() {
        const norm = this.norm();
        return new Complex(this.x / norm, this.y / norm);
    }

    // Return a new vector scaled by the given factor
    scaled(factor) {
        return new this.constructor(this.x * factor, this.y * factor);
    }

    // Return the dot product of two vectors
    dot(v) {
        return this.x * v.x + this.y * v.y;
    }

    // Return the squared norm
    sqnorm() {
        return this.dot(this);
    }

    // Return the norm
    norm() {
        return Math.sqrt(this.sqnorm());
    }

    // Compute the direct normal vector
    normal() {
        return this.rotated(new Complex(0, 1)).scaled(1 / this.norm());
    }

    // Return the negated vector
    neg() {
        return new this.constructor(-this.x, -this.y);
    }
}


// 2D point class. It inherits everything from Vector2.
export class Point2 extends Vector2 {
    // Create a 2D vector from the current point to the given one
    to(p) {
        return new Vector2(p.x - this.x, p.y - this.y);
    }

    static get origin() {
        return new Point2(0, 0);
    }
}


// Normalized complex number class. For convenience, the real part is named cos and the imaginary
// part is named sin.
export class Complex {
    constructor(cos, sin) {
        this.cos = cos;
        this.sin = sin;
    }

    // Return the negated angle
    neg() {
        return new Complex(this.cos, -this.sin);
    }

    // Return the angle in radians
    radians() {
        return Math.atan2(this.sin, this.cos);
    }
}


// 2D angle class
export class Angle2 extends Complex {
    constructor(theta) {
        super(Math.cos(theta), Math.sin(theta));
    }
}


// Shape class
class Shape {
    // Update a point in the shape
    update(i, p) {
        this.points[i] = p;
    }
}


// 2D line class
export class Line2 extends Shape {
    constructor(p, o) {
        super();
        this.p = p;
        if (o instanceof Point2) {
            this.o = o;
            this.v = p.to(o);
        }
        else {
            this.o = p.plus(o);
            this.v = o;
        }
        this.points = [p, o];
    }

    // Compute an internal projection factor
    _projectionFactor(p) {
        return this.p.to(p).dot(this.v) / this.v.sqnorm();
    }

    // Compute the orthogonal projection of a point onto the line
    project(p) {
        const pos = this._projectionFactor(p);
        return this.p.plus(this.v.scaled(pos));
    }

    // Compute the minimal distance between a point and the line
    distance(p) {
        return this.project(p).to(p).norm();
    }

    // Determine whether an intersection factor is valid
    _validIntersectionFactor(f) {
        return true;
    }

    // Compute the intersection with another line
    intersect(l) {
        const den = this.v.cross(l.v);
        const v = this.p.to(l.p).scaled(1 / den);
        const factor = v.cross(l.v);
        if (!this._validIntersectionFactor(factor) || !l._validIntersectionFactor(v.cross(this.v)))
            return null;
        return this.p.plus(this.v.scaled(factor));
    }

    // Checks whether this line intersects with another
    intersects(l) {
        const den = this.v.cross(l.v);
        const v = this.p.to(l.p).scaled(1 / den);
        return this._validIntersectionFactor(v.cross(l.v)) && l._validIntersectionFactor(v.cross(this.v));
    }

    // Update a point in the line
    update(i, p) {
        super.update(i, p);
        switch (i) {
            case 0:
                this.p = p;
                break;
            case 1:
                this.o = p;
                break;
        }
        this.v = this.p.to(this.o);
    }
}


// 2D ray class
export class Ray2 extends Line2 {
    // Compute the orthogonal projection of a point onto the ray
    project(p, clip=false) {
        const pos = this._projectionFactor(p);
        if (pos <= 0) {
            if (clip)
                return this.p;
            return null;
        }
        return this.p.plus(this.v.scaled(pos));
    }

    // Compute the minimal distance between a point and the ray
    distance(p) {
        return this.project(p, true).to(p).norm();
    }

    // Determine whether an intersection factor is valid
    _validIntersectionFactor(f) {
        return f >= 0;
    }
}


// 2D segment class
export class Segment2 extends Ray2 {
    // Compute the orthogonal projection of a point onto the segment
    project(p, clip=false) {
        const pos = this._projectionFactor(p);
        if (pos <= 0) {
            if (clip)
                return this.p;
            return null;
        }
        if (pos >= 1) {
            if (clip)
                return this.o;
            return null;
        }
        return this.p.plus(this.v.scaled(pos));
    }

    // Determine whether an intersection factor is valid
    _validIntersectionFactor(f) {
        return f >= 0 && f <= 1;
    }
}


// 2D matrix class
export class Matrix2 {
    constructor(a, b, c, d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    appliedTo(v) {
        return new v.constructor(this.a * v.x + this.b * v.y, this.c * v.x + this.d * v.y);
    }
}


// 3D point class
export class Point3 {
    constructor(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}


// 3D vector class
export class Vector3 {
    constructor(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // Return the dot product of two vectors
    dot(v) {
        return this.x * v.x + this.y * v.y + this.z * v.z;
    }

    // Return the norm
    norm() {
        return Math.sqrt(this.dot(this));
    }
}


// 2D bounding box class
export class BoundingBox2 {
    constructor(min, max) {
        this.min = min;
        this.max = max;
    }

    // Return the box width
    width() {
        return this.max.x - this.min.x;
    }

    // Return the box height
    height() {
        return this.max.y - this.min.y;
    }

    // Return the box center
    center() {
        return new Point2((this.min.x + this.max.x) / 2, (this.min.y + this.max.y) / 2);
    }

    // Check whether this box contains another
    contains(b) {
        return this.min.x <= b.min.x && this.min.y <= b.min.y && this.max.x >= b.max.x && this.max.y >= b.max.y;
    }
}


// 2D polygon class
export class Polygon2 extends Shape {
    #open;
    #selfIntersecting;
    #boundingBox;
    #edges;

    constructor(points, open=false) {
        super();
        this.points = points;
        this.#open = open;
        this.#reset();
        this.#reindex();
    }

    // Reset the polygon state
    #reset() {
        if (this.points.length < 4)
            this.#selfIntersecting = false;
        else
            this.#selfIntersecting = null;
        this.#boundingBox = null;
        this.#edges = null;
    }

    // Reindex the points
    #reindex() {
        let index = 0;
        this.points.forEach(p => p.index = index++);
    }

    // Return the polygon area, computed with the shoolace algorithm
    area() {
        let vectors = [];
        const nPoints = this.points.length;
        for (let i = 0; i < nPoints; i++)
            vectors.push(this.points[i].to(this.points[(i+1) % nPoints]));
        let area = 0;
        for (let i = 0; i < nPoints; i++)
            area += vectors[i].cross(vectors[(i+1) % nPoints]);
        return area / 2;
    }

    // Return a new rotated polygon around the origin
    rotated(angle) {
        return new Polygon2(this.points.map(p => p.rotated(angle)));
    }

    // Return the bounding box of the polygon
    boundingBox() {
        if (this.#boundingBox !== null)
            return this.#boundingBox;

        let xmin, xmax, ymin, ymax;
        for (const {x, y} of this.points) {
            if (!(x >= xmin))
                xmin = x;
            if (!(x <= xmax))
                xmax = x;
            if (!(y >= ymin))
                ymin = y;
            if (!(y <= ymax))
                ymax = y;
        }
        return new BoundingBox2(new Point2(xmin, ymin), new Point2(xmax, ymax));
    }

    // Return the polygon’s edges
    edges() {
        if (this.#edges !== null)
            return this.#edges;

        this.#edges = [];
        for (let i = 0; i < this.points.length - 1; i++)
            this.#edges.push(new Segment2(this.points[i], this.points[i+1]));
        if (!this.#open)
            this.#edges.push(new Segment2(this.points.at(-1), this.points[0]));
        return this.#edges;
    }

    // Return whether the polygon is self-intersecting. The algorithm used here is pretty expensive
    // but avoids painful precision checks.
    isSelfIntersecting() {
        if (this.#selfIntersecting !== null)
            return this.#selfIntersecting;

        const edges = this.edges();
        const length = edges.length;

        // We iterate over the edges, except the last two
        for (let i = 0; i < length - 2; i++) {
            // We iterate over the edges from i+2 to the last one (except at the first iteration)
            const bound = !this.#open && i === 0 ? length - 1 : length;
            for (let j = i + 2; j < bound; j++) {
                if (edges[i].intersects(edges[j])) {
                    this.#selfIntersecting = true;
                    return true;
                }
            }
        }
        return false;
    }

    // Return the polygon’s winding number for a given point
    windingNumber(p) {
        let wn = 0;
        let prev = this.points.at(-1);
        for (const curr of this.points) {
            if (prev.y <= p.y) {
                if (curr.y > p.y && prev.to(curr).cross(prev.to(p)) > 0)
                    wn++;
            }
            else if (curr.y <= p.y) {
                if (prev.to(curr).cross(prev.to(p)) < 0)
                    wn--;
            }
            prev = curr;
        }
        return wn;
    }

    // Checks whether this polygon intersects with another
    intersects(p) {
        for (const edge of this.edges())
            for (const edge2 of p.edges())
                if (edge.intersects(edge2))
                    return true;
        return false;
    }

    // Update a point in the polygon
    update(i, p) {
        super.update(i, p);
        this.points[i].index = i;
        this.#reset();
    }

    // Insert a point in the polygon
    insert(i, p) {
        this.points.splice(i, 0, p);
        this.#reset();
        this.#reindex();
    }

    // Remove a point from the polygon
    remove(i) {
        this.points.splice(i, 1);
        this.#reset();
        this.#reindex();
    }
}


// Bounded surface in a 3D space class. The class represents a polygon multiplied by a 3D matrix.
export class BoundedSurface3 {
    #angle;
    #boundingBox;
    #matrix;
    #normalizedPolygon;
    #polygon;

    constructor(polygon, matrix) {
        // The initial polygon is in the relative (z, x) space.
        this.#polygon = polygon;
        // WebXR guarantees that the polygon sits on the (y=0) plane in the relative reference
        // space. The rotation matrix’ second column is thus our normal vector. Let us stay in the
        // (z, x, [y]) space for convenience.
        this.#matrix = matrix;
        this.normal = new Vector3(matrix[4], matrix[5], matrix[6]);
        // We want to rotate our polygon so that their x components are orthogonal to the global y
        // axis.
        const norm = Math.sqrt(this.normal.x * this.normal.x + this.normal.z * this.normal.z)
        this.#angle = new Complex(this.normal.x / norm, -this.normal.z / norm);
        this.#normalizedPolygon = polygon.rotated(this.#angle);
    }

    // Return the surface area
    area() {
        return this.#polygon.area();
    }

    // Return the surface height on the normalized projection of the reference space’s y axis
    height() {
        this.#boundingBox ??= this.#normalizedPolygon.boundingBox();
        return this.#boundingBox.height();
    }

    // Return the projection on the y=0 reference plane of the bisector of the surface bounding box
    lineProjection() {
        this.#boundingBox ??= this.#normalizedPolygon.boundingBox();
        const midY = (this.#boundingBox.min.y + this.#boundingBox.max.y) / 2;
        const neg = this.#angle.neg();
        const l = new Point2(this.#boundingBox.min.x, midY).rotated(neg);
        const r = new Point2(this.#boundingBox.max.x, midY).rotated(neg);
        const p1 = new Point3(this.#matrix[0] * l.y + this.#matrix[8] * l.x + this.#matrix[12], 0,
                              this.#matrix[2] * l.y + this.#matrix[10] * l.x + this.#matrix[14]);
        const p2 = new Point3(this.#matrix[0] * r.y + this.#matrix[8] * r.x + this.#matrix[12], 0,
                              this.#matrix[2] * r.y + this.#matrix[10] * r.x + this.#matrix[14]);
        return [p1, p2];
    }
}


// Normalized quaternion class
export class Quaternion {
    #w;
    #x;
    #y;
    #z;

    constructor(w, x, y, z) {
        this.#w = w;
        this.#x = x;
        this.#y = y;
        this.#z = z;
    }

    // Convert quaternion to yaw (adding a pi/2 offset)
    yaw() {
        const cos = -2 * (this.#w * this.#y + this.#x * this.#z);
        const sin = 1 - 2 * (this.#y * this.#y + this.#z * this.#z);
        const norm = Math.sqrt(cos * cos + sin * sin);
        return new Complex(cos / norm, sin / norm);
    }
}
