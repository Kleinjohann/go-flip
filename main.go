package main

import (
	"flag"
	"fmt"
	"image/color"
	"log"
	"math"
	"math/rand"
	"os"
	"runtime/pprof"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
)

const (
	screenWidth  = 640
	screenHeight = 640
	particleSize = 3

	gridWidth     = 64
	gridHeight    = 64
	numParticles  = gridWidth * gridHeight * 4
	initMinHeight = 0.5
	picRatio      = 0.05
	flipRatio     = 1 - picRatio

	maxPlacementAttempts = 10000

	gravity  = 49
	timeStep = 0.01

	collisionEpsilon = 1e-4
	maxContacts      = 4

	clearance = 0.4
	separationEpsilon = 1e-4
	maxCorrections = 8

	solverTolerance  = 0.000001
	solverIterations = 1000

	widthScale  = screenWidth / gridWidth
	heightScale = screenHeight / gridHeight

	neighbourCountMask = 0b00000111
	leftNeighbour      = 1 << 3
	rightNeighbour     = 1 << 4
	topNeighbour       = 1 << 5
	bottomNeighbour    = 1 << 6
)

type ParticleSystem struct {
	positions  [numParticles][2]float64
	velocities [numParticles][2]float64
}

type CellContent int

const (
	EMPTY CellContent = iota
	FLUID
	SOLID
)

type Grid struct {
	width, height                    int
	cellContents                     [gridWidth][gridHeight]CellContent
	neighbourInfo                    [gridWidth][gridHeight]uint8
	cellPressures                    [gridWidth][gridHeight]float64
	cellDivergences                  [gridWidth][gridHeight]float64
	horizontalEdgeVelocities         [gridWidth + 1][gridHeight]float64
	verticalEdgeVelocities           [gridWidth][gridHeight + 1]float64
	horizontalEdgeVelocityWeights    [gridWidth + 1][gridHeight]float64
	verticalEdgeVelocityWeights      [gridWidth][gridHeight + 1]float64
	previousHorizontalEdgeVelocities [gridWidth + 1][gridHeight]float64
	previousVerticalEdgeVelocities   [gridWidth][gridHeight + 1]float64
	particles                        ParticleSystem
}

func (g *Grid) isSolidCell(x, y int) bool {
	if x < 0 || x >= gridWidth || y < 0 || y >= gridHeight {
		return true
	}
	return g.cellContents[x][y] == SOLID
}

func (g *Grid) Splat(x, y, vx, vy float64) {
	cellX := int(x)
	cellY := int(y)
	if g.isSolidCell(cellX, cellY) {
		panic("attempted to splat a particle inside a solid cell")
	}
	g.cellContents[cellX][cellY] = FLUID
	g.cellContents[cellX][cellY] = FLUID
	weightX := x - float64(cellX)
	weightY := y - float64(cellY)
	shiftedX := x - 0.5
	shiftedY := y - 0.5
	shiftedCellX := int(shiftedX)
	shiftedCellY := int(shiftedY)
	shiftedWeightX := shiftedX - float64(shiftedCellX)
	shiftedWeightY := shiftedY - float64(shiftedCellY)

	lowerLeftWeight := (1 - weightX) * (1 - shiftedWeightY)
	lowerRightWeight := weightX * (1 - shiftedWeightY)
	upperLeftWeight := (1 - weightX) * shiftedWeightY
	upperRightWeight := weightX * shiftedWeightY

	g.horizontalEdgeVelocities[cellX][shiftedCellY] += lowerLeftWeight * vx
	g.horizontalEdgeVelocities[cellX+1][shiftedCellY] += lowerRightWeight * vx
	g.horizontalEdgeVelocities[cellX][shiftedCellY+1] += upperLeftWeight * vx
	g.horizontalEdgeVelocities[cellX+1][shiftedCellY+1] += upperRightWeight * vx
	g.horizontalEdgeVelocityWeights[cellX][shiftedCellY] += lowerLeftWeight
	g.horizontalEdgeVelocityWeights[cellX+1][shiftedCellY] += lowerRightWeight
	g.horizontalEdgeVelocityWeights[cellX][shiftedCellY+1] += upperLeftWeight
	g.horizontalEdgeVelocityWeights[cellX+1][shiftedCellY+1] += upperRightWeight

	lowerLeftWeight = (1 - shiftedWeightX) * (1 - weightY)
	lowerRightWeight = shiftedWeightX * (1 - weightY)
	upperLeftWeight = (1 - shiftedWeightX) * weightY
	upperRightWeight = shiftedWeightX * weightY

	g.verticalEdgeVelocities[shiftedCellX][cellY] += lowerLeftWeight * vy
	g.verticalEdgeVelocities[shiftedCellX+1][cellY] += lowerRightWeight * vy
	g.verticalEdgeVelocities[shiftedCellX][cellY+1] += upperLeftWeight * vy
	g.verticalEdgeVelocities[shiftedCellX+1][cellY+1] += upperRightWeight * vy
	g.verticalEdgeVelocityWeights[shiftedCellX][cellY] += lowerLeftWeight
	g.verticalEdgeVelocityWeights[shiftedCellX+1][cellY] += lowerRightWeight
	g.verticalEdgeVelocityWeights[shiftedCellX][cellY+1] += upperLeftWeight
	g.verticalEdgeVelocityWeights[shiftedCellX+1][cellY+1] += upperRightWeight
}

func (g *Grid) ClampSolidCells() {
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight; j++ {
			if g.cellContents[i][j] == SOLID {
				g.horizontalEdgeVelocities[i][j] = 0
				g.horizontalEdgeVelocities[i+1][j] = 0
				g.verticalEdgeVelocities[i][j] = 0
				g.verticalEdgeVelocities[i][j+1] = 0
			}
		}
	}
}

func (g *Grid) ParticlesToGrid() {
	var x, y, vx, vy float64
	g.horizontalEdgeVelocities = [gridWidth + 1][gridHeight]float64{}
	g.verticalEdgeVelocities = [gridWidth][gridHeight + 1]float64{}
	g.horizontalEdgeVelocityWeights = [gridWidth + 1][gridHeight]float64{}
	g.verticalEdgeVelocityWeights = [gridWidth][gridHeight + 1]float64{}
	for i := 1; i < gridWidth-1; i++ {
		for j := 1; j < gridHeight-1; j++ {
			if g.cellContents[i][j] == FLUID {
				g.cellContents[i][j] = EMPTY
			}
		}
	}
	for i := 0; i < numParticles; i++ {
		x = g.particles.positions[i][0]
		y = g.particles.positions[i][1]
		vx = g.particles.velocities[i][0]
		vy = g.particles.velocities[i][1]
		g.Splat(x, y, vx, vy)
	}
	for i := 0; i < gridWidth+1; i++ {
		for j := 0; j < gridHeight; j++ {
			if g.horizontalEdgeVelocityWeights[i][j] != 0 {
				g.horizontalEdgeVelocities[i][j] /= g.horizontalEdgeVelocityWeights[i][j]
			}
		}
	}
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight+1; j++ {
			if g.verticalEdgeVelocityWeights[i][j] != 0 {
				g.verticalEdgeVelocities[i][j] /= g.verticalEdgeVelocityWeights[i][j]
			}
		}
	}
	g.previousHorizontalEdgeVelocities = g.horizontalEdgeVelocities
	g.previousVerticalEdgeVelocities = g.verticalEdgeVelocities
}

func (g *Grid) CalculateDivergences() {
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight; j++ {
			g.cellDivergences[i][j] = 0
		}
	}
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight; j++ {
			if g.cellContents[i][j] == FLUID {
				g.cellDivergences[i][j] = -g.verticalEdgeVelocities[i][j+1] + g.verticalEdgeVelocities[i][j] -
					g.horizontalEdgeVelocities[i+1][j] + g.horizontalEdgeVelocities[i][j]
			}
		}
	}
}

func (g *Grid) MultiplyNeighbourMatrix(vector *[gridWidth][gridHeight]float64,
	output *[gridWidth][gridHeight]float64) {
	var numNeighbours float64
	for i := 1; i < gridWidth-1; i++ {
		for j := 1; j < gridHeight-1; j++ {
			if g.cellContents[i][j] == FLUID {
				numNeighbours = float64(g.neighbourInfo[i][j] & neighbourCountMask)
				output[i][j] = vector[i][j] * numNeighbours
				if g.neighbourInfo[i][j]&leftNeighbour != 0 {
					output[i][j] -= vector[i-1][j]
				}
				if g.neighbourInfo[i][j]&rightNeighbour != 0 {
					output[i][j] -= vector[i+1][j]
				}
				if g.neighbourInfo[i][j]&bottomNeighbour != 0 {
					output[i][j] -= vector[i][j-1]
				}
				if g.neighbourInfo[i][j]&topNeighbour != 0 {
					output[i][j] -= vector[i][j+1]
				}
			}
		}
	}
}

func scalarProduct(a *[gridWidth][gridHeight]float64, b *[gridWidth][gridHeight]float64) float64 {
	var result float64
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight; j++ {
			result += a[i][j] * b[i][j]
		}
	}
	return result
}

func (g *Grid) SolvePressure() {
	var residual [gridWidth][gridHeight]float64
	var stepDirection [gridWidth][gridHeight]float64
	var sigma, sigmaOld, relTolerance, alpha, beta float64
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight; j++ {
			g.cellPressures[i][j] = 0
			stepDirection[i][j] = 0
			if g.cellContents[i][j] == FLUID {
				residual[i][j] = g.cellDivergences[i][j]
			}
		}
	}
	sigma = scalarProduct(&residual, &residual)
	if sigma == 0 {
		return
	}
	relTolerance = solverTolerance * sigma
	for k := 0; k < solverIterations; k++ {
		g.MultiplyNeighbourMatrix(&g.cellDivergences, &stepDirection)
		alpha = sigma / scalarProduct(&g.cellDivergences, &stepDirection)
		if isnan(alpha) {
			print("solver diverged after ", k, " iterations\n")
			panic("solver diverged")
		}
		for i := 1; i < gridWidth-1; i++ {
			for j := 1; j < gridHeight-1; j++ {
				if g.cellContents[i][j] == FLUID {
					g.cellPressures[i][j] += alpha * g.cellDivergences[i][j]
					residual[i][j] -= alpha * stepDirection[i][j]
				}
			}
		}
		sigmaOld = sigma
		sigma = scalarProduct(&residual, &residual)
		if sigma < relTolerance {
			return
		}
		beta = sigma / sigmaOld
		for i := 1; i < gridWidth-1; i++ {
			for j := 1; j < gridHeight-1; j++ {
				if g.cellContents[i][j] == FLUID {
					g.cellDivergences[i][j] = residual[i][j] + beta*g.cellDivergences[i][j]
				}
			}
		}
	}
	print("solver did not converge after ", solverIterations, " iterations\n")
}

func isnan(x float64) bool {
	return x != x
}

func (g *Grid) ClampPressure() {
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight; j++ {
			if g.cellPressures[i][j] < 0 {
				g.cellPressures[i][j] = 0
			}
		}
	}
}

func (g *Grid) ApplyPressure() {
	var pressure float64
	for i := 1; i < gridWidth; i++ {
		for j := 1; j < gridHeight; j++ {
			if g.cellContents[i][j] == SOLID {
				continue
			}
			pressure = 0.0
			if g.cellContents[i][j] == FLUID {
				pressure = g.cellPressures[i][j]
			}
			if g.cellContents[i-1][j] == FLUID {
				g.horizontalEdgeVelocities[i][j] -= (pressure - g.cellPressures[i-1][j])
			} else if g.cellContents[i-1][j] == EMPTY {
				g.horizontalEdgeVelocities[i][j] -= pressure
			}
			if g.cellContents[i][j-1] == FLUID {
				g.verticalEdgeVelocities[i][j] -= (pressure - g.cellPressures[i][j-1])
			} else if g.cellContents[i][j-1] == EMPTY {
				g.verticalEdgeVelocities[i][j] -= pressure
			}
		}
	}
}

func (g *Grid) UpdateNeighbourInfo() {
	for i := 1; i < gridWidth-1; i++ {
		for j := 1; j < gridHeight-1; j++ {
			if g.cellContents[i][j] != FLUID {
				continue
			}
			g.neighbourInfo[i][j] = 4
			if g.cellContents[i-1][j] == SOLID {
				g.neighbourInfo[i][j] -= 1
			} else if g.cellContents[i-1][j] == FLUID {
				g.neighbourInfo[i][j] |= leftNeighbour
			}
			if g.cellContents[i+1][j] == SOLID {
				g.neighbourInfo[i][j] -= 1
			} else if g.cellContents[i+1][j] == FLUID {
				g.neighbourInfo[i][j] |= rightNeighbour
			}
			if g.cellContents[i][j-1] == SOLID {
				g.neighbourInfo[i][j] -= 1
			} else if g.cellContents[i][j-1] == FLUID {
				g.neighbourInfo[i][j] |= bottomNeighbour
			}
			if g.cellContents[i][j+1] == SOLID {
				g.neighbourInfo[i][j] -= 1
			} else if g.cellContents[i][j+1] == FLUID {
				g.neighbourInfo[i][j] |= topNeighbour
			}
		}
	}
}

func sampleBilinearValid(
	x, y float64,
	sample func(i, j int) (float64, bool),
) float64 {
	i0 := int(math.Floor(x))
	j0 := int(math.Floor(y))

	fx := x - float64(i0)
	fy := y - float64(j0)

	wx := [2]float64{1 - fx, fx}
	wy := [2]float64{1 - fy, fy}

	sum := 0.0
	weightSum := 0.0

	for di := 0; di < 2; di++ {
		for dj := 0; dj < 2; dj++ {
			w := wx[di] * wy[dj]
			if w == 0 {
				continue
			}

			value, valid := sample(i0+di, j0+dj)
			if !valid {
				continue
			}

			sum += w * value
			weightSum += w
		}
	}

	if weightSum == 0 {
		return 0
	}
	return sum / weightSum
}

func (g *Grid) interpolateGridVelocities(
	x, y float64,
	horizontal *[gridWidth + 1][gridHeight]float64,
	vertical *[gridWidth][gridHeight + 1]float64,
) (float64, float64) {
	// Horizontal velocity samples live at (i, j+0.5).
	vx := sampleBilinearValid(x, y-0.5,
		func(i, j int) (float64, bool) {
			if i < 0 || i > gridWidth ||
				j < 0 || j >= gridHeight {
				return 0, false
			}

			// Exclude solid-interior faces, not solid-fluid faces.
			if g.isSolidCell(i-1, j) && g.isSolidCell(i, j) {
				return 0, false
			}

			return horizontal[i][j], true
		},
	)

	// Vertical velocity samples live at (i+0.5, j).
	vy := sampleBilinearValid(x-0.5, y,
		func(i, j int) (float64, bool) {
			if i < 0 || i >= gridWidth ||
				j < 0 || j > gridHeight {
				return 0, false
			}

			if g.isSolidCell(i, j-1) && g.isSolidCell(i, j) {
				return 0, false
			}

			return vertical[i][j], true
		},
	)

	return vx, vy
}

func sign(x int) int {
	if x < 0 {
		return -1
	} else if x > 0 {
		return 1
	} else {
		return 0
	}
}

func abs(x int) int {
	if x < 0 {
		return -x
	} else {
		return x
	}
}

func (g *Grid) sweepParticle(
	x, y, dx, dy float64,
) (float64, float64, float64, bool) {
	cellX := int(math.Floor(x))
	cellY := int(math.Floor(y))

	if g.isSolidCell(cellX, cellY) {
		panic("particle starts inside a solid cell")
	}

	stepX, stepY := 0, 0

	// Segment parameters at the next vertical/horizontal grid boundary.
	tMaxX := math.Inf(1)
	tMaxY := math.Inf(1)

	// Parameter increments between successive boundaries.
	tDeltaX := math.Inf(1)
	tDeltaY := math.Inf(1)

	if dx > 0 {
		stepX = 1
		tMaxX = (float64(cellX+1) - x) / dx
		tDeltaX = 1 / dx
	} else if dx < 0 {
		stepX = -1
		tMaxX = (float64(cellX) - x) / dx
		tDeltaX = -1 / dx
	}

	if dy > 0 {
		stepY = 1
		tMaxY = (float64(cellY+1) - y) / dy
		tDeltaY = 1 / dy
	} else if dy < 0 {
		stepY = -1
		tMaxY = (float64(cellY) - y) / dy
		tDeltaY = -1 / dy
	}

	for {
		t := math.Min(tMaxX, tMaxY)

		// No more grid boundaries within this displacement.
		// Both values are infinite when dx == dy == 0.
		if t > 1 || math.IsInf(t, 1) {
			return 1, 0, 0, false
		}

		// Both are true when crossing an exact grid corner.
		crossX := tMaxX <= tMaxY
		crossY := tMaxY <= tMaxX

		nextX, nextY := cellX, cellY
		if crossX {
			nextX += stepX
		}
		if crossY {
			nextY += stepY
		}

		blockX := crossX && g.isSolidCell(nextX, cellY)
		blockY := crossY && g.isSolidCell(cellX, nextY)

		// At a corner, the diagonal cell can be solid even if
		// the two side cells are empty. Conservatively stop both axes.
		if !blockX && !blockY && g.isSolidCell(nextX, nextY) {
			blockX = crossX
			blockY = crossY
		}

		if blockX || blockY {
			nx, ny := 0.0, 0.0
			if blockX {
				nx = -float64(stepX)
			}
			if blockY {
				ny = -float64(stepY)
			}
			return t, nx, ny, true
		}

		cellX, cellY = nextX, nextY

		if crossX {
			tMaxX += tDeltaX
		}
		if crossY {
			tMaxY += tDeltaY
		}
	}
}

func (g *Grid) KeepParticleAwayFromSolids(i int) {
	p := &g.particles.positions[i]
	v := &g.particles.velocities[i]

	for correction := 0; correction < maxCorrections; correction++ {
		cx := int(math.Floor(p[0]))
		cy := int(math.Floor(p[1]))

		// The swept collision routine should already prevent penetration.
		if g.isSolidCell(cx, cy) {
			panic("clearance correction: particle is inside solid")
		}

		deepest := 0.0
		contactNX, contactNY := 0.0, 0.0

		// A 3x3 neighborhood suffices for clearance < 0.5 cells.
		for sx := cx - 1; sx <= cx+1; sx++ {
			for sy := cy - 1; sy <= cy+1; sy++ {
				if !g.isSolidCell(sx, sy) {
					continue
				}

				minX := float64(sx)
				minY := float64(sy)
				maxX := minX + 1
				maxY := minY + 1

				// Closest point on this solid cell's rectangle.
				qx := math.Max(minX, math.Min(p[0], maxX))
				qy := math.Max(minY, math.Min(p[1], maxY))

				dx := p[0] - qx
				dy := p[1] - qy
				distance := math.Hypot(dx, dy)

				if distance >= clearance {
					continue
				}

				nx, ny := 0.0, 0.0

				if distance > 0 {
					nx = dx / distance
					ny = dy / distance
				} else {
					// Exact contact with the rectangle boundary.
					// Interior positions were rejected above.
					switch {
					case p[0] <= minX:
						nx = -1
					case p[0] >= maxX:
						nx = 1
					case p[1] <= minY:
						ny = -1
					case p[1] >= maxY:
						ny = 1
					default:
						panic("particle penetrated a solid rectangle")
					}
				}

				penetration := clearance - distance
				if penetration > deepest {
					deepest = penetration
					contactNX = nx
					contactNY = ny
				}
			}
		}

		if deepest == 0 {
			return
		}

		// Resolve the strongest contact, then search again.
		// Searching again handles contact with multiple walls.
		p[0] += (deepest + separationEpsilon) * contactNX
		p[1] += (deepest + separationEpsilon) * contactNY

		// Remove only velocity directed toward the solid.
		vn := v[0]*contactNX + v[1]*contactNY
		if vn < 0 {
			v[0] -= vn * contactNX
			v[1] -= vn * contactNY
		}
	}

	// Useful during debugging. Difficult/narrow geometry may require
	// a smaller clearance or a more sophisticated contact solver.
	panic("solid clearance correction did not converge")
}

func (g *Grid) UpdateParticles() {
	// Grid-cell units. This is a numerical clearance, not a particle radius.

	for i := 0; i < numParticles; i++ {
		x := g.particles.positions[i][0]
		y := g.particles.positions[i][1]

		vx := g.particles.velocities[i][0]
		vy := g.particles.velocities[i][1]

		remainingTime := timeStep

		for contact := 0; contact < maxContacts && remainingTime > 0; contact++ {
			dx := vx * remainingTime
			dy := vy * remainingTime

			t, nx, ny, hit := g.sweepParticle(x, y, dx, dy)

			if !hit {
				x += dx
				y += dy
				remainingTime = 0
				break
			}

			// Move to contact, then slightly back into free space.
			x += t*dx + nx*collisionEpsilon
			y += t*dy + ny*collisionEpsilon

			// Frictionless, non-bouncing contact:
			// remove only velocity directed into the wall.
			if nx != 0 && vx*nx < 0 {
				vx = 0
			}
			if ny != 0 && vy*ny < 0 {
				vy = 0
			}

			remainingTime *= 1 - t
		}

		// If the contact limit is reached, discard the remaining movement
		// rather than risking penetration.
		g.particles.positions[i] = [2]float64{x, y}
		g.particles.velocities[i] = [2]float64{vx, vy}

		g.KeepParticleAwayFromSolids(i)
	}
}

func (g *Grid) ApplyGravity() {
	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight+1; j++ {
			g.verticalEdgeVelocities[i][j] -= gravity * timeStep
		}
	}
}

func (g *Grid) MakeIncompressible() {
	g.CalculateDivergences()
	g.SolvePressure()
	// g.ClampPressure()
	g.ApplyPressure()
}

func (g *Grid) GridToParticles() {
	for i := 0; i < numParticles; i++ {
		x := g.particles.positions[i][0]
		y := g.particles.positions[i][1]

		oldGridX, oldGridY := g.interpolateGridVelocities(
			x, y,
			&g.previousHorizontalEdgeVelocities,
			&g.previousVerticalEdgeVelocities,
		)

		picX, picY := g.interpolateGridVelocities(
			x, y,
			&g.horizontalEdgeVelocities,
			&g.verticalEdgeVelocities,
		)

		flipX := g.particles.velocities[i][0] + picX - oldGridX
		flipY := g.particles.velocities[i][1] + picY - oldGridY

		g.particles.velocities[i][0] =
			picRatio*picX + flipRatio*flipX
		g.particles.velocities[i][1] =
			picRatio*picY + flipRatio*flipY
	}
}

func (g *Grid) Update() {
	g.ParticlesToGrid()
	g.UpdateNeighbourInfo()
	g.ApplyGravity()
	g.ClampSolidCells()
	g.MakeIncompressible()
	g.GridToParticles()
	g.UpdateParticles()
}

var (
	particleSprite = ebiten.NewImage(particleSize, particleSize)
	cellSprite     = map[CellContent]*ebiten.Image{
		EMPTY: ebiten.NewImage(int(widthScale), int(heightScale)),
		FLUID: ebiten.NewImage(int(widthScale), int(heightScale)),
		SOLID: ebiten.NewImage(int(widthScale), int(heightScale)),
	}
)

func init() {
	particleSprite.Fill(color.White)
	cellSprite[EMPTY].Fill(color.Black)
	cellSprite[FLUID].Fill(color.RGBA{0, 0, 255, 255})
	cellSprite[SOLID].Fill(color.RGBA{100, 100, 100, 255})
}

type Game struct {
	grid *Grid
	keys []ebiten.Key
}

func NewGrid(width, height int) *Grid {
	var g Grid

	for i := 0; i < width; i++ {
		g.cellContents[i][0] = SOLID
		g.cellContents[i][height-1] = SOLID
	}
	for i := 0; i < height; i++ {
		g.cellContents[0][i] = SOLID
		g.cellContents[width-1][i] = SOLID
	}
	for i := int(float64(width) * 0.4); i < int(float64(width)*0.9); i++ {
		for j := int(float64(height) * 0.4); j < int(float64(height)*0.6); j++ {
			g.cellContents[i][j] = SOLID
		}
	}

	usableWidth := width - 2
	usableHeight := height - 2

	for i := 0; i < numParticles; i++ {
		for attempt := 0; ; attempt++ {
			if attempt >= maxPlacementAttempts {
				panic("cannot place particle: initial region may be entirely solid")
			}

			x := 1 + float64(usableWidth)*
				(initMinHeight+(1-initMinHeight)*rand.Float64())
			y := 1 + float64(usableHeight)*
				(initMinHeight+(1-initMinHeight)*rand.Float64())

			// These initialization coordinates are positive and inside the grid.
			cellX := int(x)
			cellY := int(y)

			if g.cellContents[cellX][cellY] == SOLID {
				continue
			}

			g.particles.positions[i] = [2]float64{x, y}
			g.particles.velocities[i] = [2]float64{0, 0}
			break
		}
	}

	return &g
}

func NewGame() *Game {
	return &Game{
		grid: NewGrid(gridWidth, gridHeight),
		keys: []ebiten.Key{},
	}
}

func (g *Game) Update() error {
	g.keys = inpututil.AppendPressedKeys(g.keys[:0])
	if ebiten.IsKeyPressed(ebiten.KeyQ) {
		return ebiten.Termination
	}
	g.grid.Update()
	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	screen.Fill(color.Black)

	op := &ebiten.DrawImageOptions{}
	op.ColorScale.Scale(200.0/255.0, 200.0/255.0, 200.0/255.0, 1)

	for i := 0; i < gridWidth; i++ {
		for j := 0; j < gridHeight; j++ {
			x := float64(i) * widthScale
			y := float64(gridHeight-j-1) * heightScale
			op.GeoM.Reset()
			op.GeoM.Translate(x, y)
			if g.grid.cellContents[i][j] == SOLID {
				screen.DrawImage(cellSprite[SOLID], op)
			} else if g.grid.cellContents[i][j] == FLUID {
				screen.DrawImage(cellSprite[FLUID], op)
			} else {
				screen.DrawImage(cellSprite[EMPTY], op)
			}
		}
	}

	for i := 0; i < numParticles; i++ {
		x := g.grid.particles.positions[i][0] * widthScale
		y := (gridHeight - g.grid.particles.positions[i][1]) * heightScale
		op.GeoM.Reset()
		op.GeoM.Translate(x, y)
		screen.DrawImage(particleSprite, op)

	}

	ebitenutil.DebugPrint(screen, fmt.Sprintf("TPS: %0.2f", ebiten.ActualTPS()))
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return screenWidth, screenHeight
}

func main() {
	var (
		cpuprofile = flag.String("cpuprofile", "", "write cpu profile to `file`")
	)
	flag.Usage = func() {
		fmt.Fprintf(flag.CommandLine.Output(),
			"Usage: go-flip [-cpuprofile <file>]\n")
		flag.PrintDefaults()
	}
	flag.Parse()
	if *cpuprofile != "" {
		f, err := os.Create(*cpuprofile)
		if err != nil {
			log.Fatal("could not create CPU profile: ", err)
		}
		defer f.Close()
		if err := pprof.StartCPUProfile(f); err != nil {
			log.Fatal("could not start CPU profile: ", err)
		}
		defer pprof.StopCPUProfile()
	}

	ebiten.SetWindowSize(screenWidth*1.2, screenHeight*1.2)
	ebiten.SetWindowTitle("Fluid Simulation")
	game := NewGame()
	if err := ebiten.RunGame(game); err != nil {
		log.Fatal(err)
	}
}
