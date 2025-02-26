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

    // Return a new vector scaled by the given factor
    scaled(factor) {
        return new this.constructor(this.x * factor, this.y * factor);
    }

    // Return the dot product of two vectors
    dot(v) {
        return this.x * v.x + this.y * v.y;
    }

    // Return the norm
    norm() {
        return Math.sqrt(this.dot(this));
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
}


// 2D angle class
export class Angle2 extends Complex {
    constructor(theta) {
        super(Math.cos(theta), Math.sin(theta));
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
}


// 2D polygon class
export class Polygon2 {
    #points;

    constructor(points) {
        this.#points = points;
    }

    // Return the polygon area, computed with the shoolace algorithm
    area() {
        let vectors = [];
        const nPoints = this.#points.length;
        for (let i = 0; i < nPoints; i++)
            vectors.push(this.#points[i].to(this.#points[(i+1) % nPoints]));
        let area = 0;
        for (let i = 0; i < nPoints; i++)
            area += vectors[i].cross(vectors[(i+1) % nPoints]);
        return area / 2;
    }

    // Return a new rotated polygon around the origin
    rotated(angle) {
        return new Polygon2(this.#points.map(p => p.rotated(angle)));
    }

    // Return the bounding box of the polygon
    boundingBox() {
        let xmin, xmax, ymin, ymax;
        for (const {x, y} of this.#points) {
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
