/*
 * Poly2Tri Copyright (c) 2009-2018, Poly2Tri Contributors
 * https://github.com/jhasse/poly2tri
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import { Point, pointsEqual } from './edge';

export type Triple<T> = [T, T, T];

export class Triangle {
  // Neighbour list
  private readonly neighbors: Triple<Triangle | null> = [null, null, null];

  // Triangle points
  private readonly points: Triple<Point>;

  // Flags to determine if an edge is a Constrained edge
  public readonly constrainedEdge: Triple<boolean> = [false, false, false];

  // Flags to determine if an edge is a Delauney edge
  public readonly delaunayEdge: Triple<boolean> = [false, false, false];

  // Has this triangle been marked as an interior triangle?
  private interior = false;

  // Constructor
  public constructor(a: Point, b: Point, c: Point) {
    this.points = [a, b, c];
  }

  public getPoint(index: number): Point {
    return this.points[index];
  }

  public pointCW(point: Point): Point {
    if (pointsEqual(point, this.points[0])) return this.points[2];

    if (pointsEqual(point, this.points[1])) return this.points[0];

    if (pointsEqual(point, this.points[2])) return this.points[1];

    throw new Error('Triangle::pointCW: point not part of triangle');
  }

  public pointCCW(point: Point): Point {
    if (pointsEqual(point, this.points[0])) return this.points[1];

    if (pointsEqual(point, this.points[1])) return this.points[2];

    if (pointsEqual(point, this.points[2])) return this.points[0];

    throw new Error('Triangle::pointCCW: point not part of triangle');
  }

  public oppositePoint(t: Triangle, p: Point): Point {
    const cw = t.pointCW(p);
    return this.pointCW(cw);
  }

  public getNeighbor(index: number): Triangle | null {
    return this.neighbors[index];
  }

  private markNeighborAlongEdge(p1: Point, p2: Point, t: Triangle) {
    if (
      (pointsEqual(p1, this.points[2]) && pointsEqual(p2, this.points[1])) ||
      (pointsEqual(p1, this.points[1]) && pointsEqual(p2, this.points[2]))
    )
      this.neighbors[0] = t;
    else if (
      (pointsEqual(p1, this.points[0]) && pointsEqual(p2, this.points[2])) ||
      (pointsEqual(p1, this.points[2]) && pointsEqual(p2, this.points[0]))
    )
      this.neighbors[1] = t;
    else if (
      (pointsEqual(p1, this.points[0]) && pointsEqual(p2, this.points[1])) ||
      (pointsEqual(p1, this.points[1]) && pointsEqual(p2, this.points[0]))
    )
      this.neighbors[2] = t;
    else
      throw new Error(
        'Triangle::markNeighborAlongEdge: points not part of triangle'
      );
  }

  public markNeighbor(t: Triangle) {
    if (t.containsPoints(this.points[1], this.points[2])) {
      this.neighbors[0] = t;
      t.markNeighborAlongEdge(this.points[1], this.points[2], this);
    } else if (t.containsPoints(this.points[0], this.points[2])) {
      this.neighbors[1] = t;
      t.markNeighborAlongEdge(this.points[0], this.points[2], this);
    } else if (t.containsPoints(this.points[0], this.points[1])) {
      this.neighbors[2] = t;
      t.markNeighborAlongEdge(this.points[0], this.points[1], this);
    }
  }

  public markConstrainedEdgeByIndex(index: number) {
    this.constrainedEdge[index] = true;
  }

  public markConstrainedEdge(p: Point, q: Point) {
    if (
      (pointsEqual(q, this.points[0]) && pointsEqual(p, this.points[1])) ||
      (pointsEqual(q, this.points[1]) && pointsEqual(p, this.points[0]))
    )
      this.constrainedEdge[2] = true;
    else if (
      (pointsEqual(q, this.points[0]) && pointsEqual(p, this.points[2])) ||
      (pointsEqual(q, this.points[2]) && pointsEqual(p, this.points[0]))
    )
      this.constrainedEdge[1] = true;
    else if (
      (pointsEqual(q, this.points[1]) && pointsEqual(p, this.points[2])) ||
      (pointsEqual(q, this.points[2]) && pointsEqual(p, this.points[1]))
    )
      this.constrainedEdge[0] = true;
  }

  public index(p: Point): number {
    if (pointsEqual(p, this.points[0])) return 0;

    if (pointsEqual(p, this.points[1])) return 1;

    if (pointsEqual(p, this.points[2])) return 2;

    throw new Error('Triangle::index: point not part of triangle');
  }

  public edgeIndex(p1: Point, p2: Point): number {
    if (pointsEqual(this.points[0], p1)) {
      if (pointsEqual(this.points[1], p2)) return 2;

      if (pointsEqual(this.points[2], p2)) return 1;
    } else if (pointsEqual(this.points[1], p1)) {
      if (pointsEqual(this.points[2], p2)) return 0;

      if (pointsEqual(this.points[0], p2)) return 2;
    } else if (pointsEqual(this.points[2], p1)) {
      if (pointsEqual(this.points[0], p2)) return 1;

      if (pointsEqual(this.points[1], p2)) return 0;
    }

    return -1;
  }

  public neighborAcross(point: Point): Triangle | null {
    if (pointsEqual(point, this.points[0])) return this.neighbors[0];

    if (pointsEqual(point, this.points[1])) return this.neighbors[1];

    return this.neighbors[2];
  }

  public neighborCW(point: Point): Triangle | null {
    if (pointsEqual(point, this.points[0])) return this.neighbors[1];

    if (pointsEqual(point, this.points[1])) return this.neighbors[2];

    return this.neighbors[0];
  }

  public neighborCCW(point: Point): Triangle | null {
    if (pointsEqual(point, this.points[0])) return this.neighbors[2];

    if (pointsEqual(point, this.points[1])) return this.neighbors[0];

    return this.neighbors[1];
  }

  public getConstrainedEdgeCCW(p: Point): boolean {
    if (pointsEqual(p, this.points[0])) return this.constrainedEdge[2];

    if (pointsEqual(p, this.points[1])) return this.constrainedEdge[0];

    return this.constrainedEdge[1];
  }

  public getConstrainedEdgeCW(p: Point): boolean {
    if (pointsEqual(p, this.points[0])) return this.constrainedEdge[1];

    if (pointsEqual(p, this.points[1])) return this.constrainedEdge[2];

    return this.constrainedEdge[0];
  }

  public setConstrainedEdgeCCW(p: Point, ce: boolean) {
    if (pointsEqual(p, this.points[0])) this.constrainedEdge[2] = ce;
    else if (pointsEqual(p, this.points[1])) this.constrainedEdge[0] = ce;
    else this.constrainedEdge[1] = ce;
  }

  public setConstrainedEdgeCW(p: Point, ce: boolean) {
    if (pointsEqual(p, this.points[0])) this.constrainedEdge[1] = ce;
    else if (pointsEqual(p, this.points[1])) this.constrainedEdge[2] = ce;
    else this.constrainedEdge[0] = ce;
  }

  public getDelunayEdgeCCW(p: Point): boolean {
    if (pointsEqual(p, this.points[0])) return this.delaunayEdge[2];

    if (pointsEqual(p, this.points[1])) return this.delaunayEdge[0];

    return this.delaunayEdge[1];
  }

  public getDelunayEdgeCW(p: Point): boolean {
    if (pointsEqual(p, this.points[0])) return this.delaunayEdge[1];

    if (pointsEqual(p, this.points[1])) return this.delaunayEdge[2];

    return this.delaunayEdge[0];
  }

  public setDelunayEdgeCCW(p: Point, e: boolean) {
    if (pointsEqual(p, this.points[0])) this.delaunayEdge[2] = e;
    else if (pointsEqual(p, this.points[1])) this.delaunayEdge[0] = e;
    else this.delaunayEdge[1] = e;
  }

  public setDelunayEdgeCW(p: Point, e: boolean) {
    if (pointsEqual(p, this.points[0])) this.delaunayEdge[1] = e;
    else if (pointsEqual(p, this.points[1])) this.delaunayEdge[2] = e;
    else this.delaunayEdge[0] = e;
  }

  private containsPoint(p: Point): boolean {
    return (
      pointsEqual(p, this.points[0]) ||
      pointsEqual(p, this.points[1]) ||
      pointsEqual(p, this.points[2])
    );
  }

  public containsPoints(p: Point, q: Point): boolean {
    return this.containsPoint(p) && this.containsPoint(q);
  }

  public legalize(opoint: Point, npoint: Point) {
    if (pointsEqual(opoint, this.points[0])) {
      this.points[1] = this.points[0];
      this.points[0] = this.points[2];
      this.points[2] = npoint;
    } else if (pointsEqual(opoint, this.points[1])) {
      this.points[2] = this.points[1];
      this.points[1] = this.points[0];
      this.points[0] = npoint;
    } else if (pointsEqual(opoint, this.points[2])) {
      this.points[0] = this.points[2];
      this.points[2] = this.points[1];
      this.points[1] = npoint;
    } else {
      throw new Error('Triangle::legalize: points not part of triangle');
    }
  }

  public clearNeighbors() {
    this.neighbors[0] = null;
    this.neighbors[1] = null;
    this.neighbors[2] = null;
  }

  public clearDelunayEdges() {
    this.delaunayEdge[0] = this.delaunayEdge[1] = this.delaunayEdge[2] = false;
  }

  public get isInterior(): boolean {
    return this.interior;
  }

  public set isInterior(b: boolean) {
    this.interior = b;
  }
}
