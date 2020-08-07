// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "SplineFunctionLibrary.generated.h"

#define print(text) if (GEngine) GEngine->AddOnScreenDebugMessage(-1, 1.5, FColor::White,text)

USTRUCT()
struct FControlPoint
{
	GENERATED_BODY()

	UPROPERTY()
		FVector OutTangent;

	UPROPERTY()
		FVector InTangent;

	UPROPERTY()
		FVector Position;

	FControlPoint()
	{
		Position = FVector(0, 0, 0);
		InTangent = FVector(0, 0, 0);
		OutTangent = FVector(0, 0, 0);
	}

	FControlPoint(FVector mPosition)
	{
		Position = mPosition;
		InTangent = FVector(0, 0, 0);
		OutTangent = FVector(0, 0, 0);
	}
};

/**
 * 
 */
UCLASS()
class SPLINELIBARYPLUGIN_API USplineFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

	// Binomial Cofficient is n! / (i! * (n - i)!)
	static float CalcBinomialCoefficient(int N, int i)
	{
		if (i == 0 || i == N)
			return 1;

		return Factorial(N) / (Factorial(i) * Factorial(N - i));
	}

	// this will reutrn n! 
	// if n = 5, then 5! = 5 * 4 * 3 * 2 * 1;
	static float Factorial(int N)
	{
		if (N == 1 || N == 0)
			return 1;
		
		// equals to 5 * 4!
		return N * Factorial(N - 1);
	}


	// 这个只要叫一次
	// 1. Populate Controlpoints Static Tarray
	// 2. using Controlpoints to generate new bezier
	// we can also use update bezier to modify tangent of certain point.
	static TArray<FControlPoint> ConstructPointsCatmullRom(TArray<FVector> Points, bool loop = false)
	{		
		const int32 NumPoints = Points.Num();

		TArray<FControlPoint> BezierControlPnts;

		for (int i = 0; i < Points.Num(); i++)
		{
			BezierControlPnts.Add(FControlPoint(Points[i]));
		}

		if (NumPoints < 2)
			return BezierControlPnts;

		// generate tangent for each bezier control point
		// adapt Catmull-Rom spline
		// 我又 除二为了Bezier 更Smooth， 因为两边都有tangent.
		// 最前和最后特殊处理切线
		BezierControlPnts[0].OutTangent = (BezierControlPnts[1].Position - BezierControlPnts[0].Position) * 0.5f + BezierControlPnts[0].Position;
		BezierControlPnts[0].InTangent = (BezierControlPnts[1].Position - BezierControlPnts[0].Position) * -0.5f + BezierControlPnts[0].Position;


		BezierControlPnts[NumPoints - 1].InTangent = (BezierControlPnts[NumPoints - 1].Position - BezierControlPnts[NumPoints - 2].Position) * -0.5f + BezierControlPnts[NumPoints-1].Position;
		BezierControlPnts[NumPoints - 1].OutTangent = (BezierControlPnts[NumPoints - 2].Position - BezierControlPnts[NumPoints - 1].Position)  * -0.5f + BezierControlPnts[NumPoints-1].Position;

		// 超过两个control points.
		if(NumPoints > 2)
			for (int i = 1; i < NumPoints - 1; i++)
			{
				BezierControlPnts[i].OutTangent = (BezierControlPnts[i + 1].Position - BezierControlPnts[i - 1].Position) * 0.5 /2 + BezierControlPnts[i].Position;
				BezierControlPnts[i].InTangent = (BezierControlPnts[i - 1].Position - BezierControlPnts[i + 1].Position) * 0.5 /2 + BezierControlPnts[i].Position;
			}

		return BezierControlPnts;
	}


	// careful when using this, when alot of points is been used
	// processing power goes through the roof N^2
	// this is not efficient we can use pre build cubic or quadratic with tangent control points.
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Spline Function Library")
	static FVector NthOrderBezierCurve(TArray<FVector> Points, float t)
	{
		// when you have 5 points, you are able to construct 4th order bezier
		// when you have 4 points, you are able to construct cubic bezier curve
		// therefore we need to figure out which order of bezier we are curretly constructing.

		int NthOrder = Points.Num() - 1; // this will be the current order of bezier we will be constructing.

		TArray<float> Us;
		TArray<float> Ts;

		Ts.Add(1);
		Us.Add(1);

		for (int i = 1; i <= NthOrder; i++)
		{
			Us.Add(Us[i-1] * (1-t));
			Ts.Add(Ts[i-1] * t);
		}

		// now we have both nthorder powers values that are stored in List.
		FVector CalcPoint = FVector(0,0,0);

		// next we need to calculate NthOrder powers
		// we need to find out NthOrder is even or odd		
		for (int i = 0; i < Points.Num(); i++)
		{
			int reverseOrder = NthOrder - i;

			float UTerm = Us[reverseOrder];
			float TTerm = Ts[i];

			CalcPoint += UTerm * TTerm * (CalcBinomialCoefficient(NthOrder, i)) * Points[i];
		}

		return CalcPoint;
	}
	

	// 3 阶贝塞尔曲线
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Spline Function Library")
	static FVector CubicBezierCurve(FVector P0, FVector P1, FVector P2, FVector P3, float t)
	{
		float u = 1 - t;
		float uu= u* u;
		float uuu = u * u * u;

		float tt = t * t;
		float ttt = tt * t;

		return uuu * P0 + 3 * uu * t * P1 + 3 * u * tt * P2 + ttt * P3;
	}

	// 3 阶贝塞尔曲线的 每个点的切线也就是当时那个点的速度, first derivative
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Spline Function Library")
	static FVector CubicBezierTangent(FVector P0, FVector P1, FVector P2, FVector P3, float t)
	{
		t = FMath::Clamp(t, 0.0f, 1.0f);
		float u = 1 - t;

		return 3 * u * u * (P1 - P0) + 6 * u * t * (P2 - P1) + 3 * t * t * (P3 - P2);		
	}

	// 2 阶贝塞尔曲线
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Spline Function Library")
	static FVector QuadraticBezierCurve(FVector P0, FVector P1, FVector P2, float t)
	{
		float u = 1 - t;
		float uu = u * u;
		float tt = t * t;

		return uu * P0 + 2 * u * t * P1 + tt * P2;
	}

	// 2 阶贝塞尔曲线的 每个点的切线也就是当时那个点的速度, first derivative
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Spline Function Library")
	static FVector QuadraticBezierTangent(FVector P0, FVector P1, FVector P2, float t)
	{
		float u = 1 - t;

		return 2 * u * (P1 - P0) + 2 * t * (P2 - P1);
	}

	// 1 阶贝塞尔曲线
	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Spline Function Library")
	static FVector LinearBezierCurve(FVector P0, FVector P1, float t)
	{
		return (1 - t)* P0 + t * P1; // this is just linear interpolation
	}


	// Hermite Curve
	static FVector HermiteCurve(FVector M0, FVector M1, FVector P0, FVector P1, float t)
	{
		FVector Tangent0 = M0 - P0;
		FVector Tangent1 = M1 - P1;

		float tt = t * t;
		float ttt = t * t * t;

		return (2.0f * ttt - 3.0f * tt + 1.0f) * P0 + (ttt - 2.0 * tt + t) * M0 + (-2.0 * ttt + 3.0f * tt) * P1 + (ttt - tt) * Tangent1;

	}


	UFUNCTION(BlueprintCallable, Category = "Spline Function Library")
	static void ConstructBezierCurveCatmullRom(AActor* Actor, TArray<FVector> Points, bool isClosed)
	{
		UWorld* WorldPtr = Actor->GetWorld();

		if (WorldPtr == nullptr)
			return;

		TArray<FControlPoint> HandlePoints = ConstructPointsCatmullRom(Points); // construct handle points for all points using Catmull-Rom


		DebugDrawOutBezier(Actor, HandlePoints);
	}

	static void DebugDrawOutBezier(AActor* Actor, TArray<FControlPoint> BezierControlPnts) // we can modify it later, testing for now.
	{
		UWorld* WorldPtr = Actor->GetWorld();
		
		if (WorldPtr == nullptr)
			return;


		// draw first tangent
		DrawDebugSphere(WorldPtr, BezierControlPnts[0].OutTangent, 1, 100, FColor::Red);
		DrawDebugLine(WorldPtr, BezierControlPnts[0].Position, BezierControlPnts[0].OutTangent, FColor::Blue);

		FVector TempPos = BezierControlPnts[0].Position;

		// draw how many res
		for (int j = 0; j < 100; j++)
		{
			float t = (float)j / (float)100;

			FVector NextPosition = CubicBezierCurve(BezierControlPnts[0].Position, BezierControlPnts[0].OutTangent, BezierControlPnts[1].InTangent, BezierControlPnts[1].Position, t);

			DrawDebugLine(WorldPtr, TempPos, NextPosition, FColor::Emerald);

			TempPos = NextPosition;
		}


		DrawDebugSphere(WorldPtr, BezierControlPnts[BezierControlPnts.Num() - 1].OutTangent, 1, 100, FColor::Red);
		DrawDebugLine(WorldPtr, BezierControlPnts[BezierControlPnts.Num() - 1].Position, BezierControlPnts[BezierControlPnts.Num() - 1].OutTangent, FColor::Blue);


		TempPos = BezierControlPnts[BezierControlPnts.Num() - 2].Position;
		// draw how many res
		for (int j = 0; j < 100; j++)
		{
			float t = (float)j / (float)100;

			FVector NextPosition = CubicBezierCurve(BezierControlPnts[BezierControlPnts.Num() - 2].Position, BezierControlPnts[BezierControlPnts.Num() - 2].OutTangent, BezierControlPnts[BezierControlPnts.Num() - 1].InTangent, BezierControlPnts[BezierControlPnts.Num() - 1].Position, t);

			DrawDebugLine(WorldPtr, TempPos, NextPosition, FColor::Emerald);

			TempPos = NextPosition;
		}

		
		// we have constructed our points, now let's draw out bezier segement by segment
		for (int i = 1; i < BezierControlPnts.Num() - 1; i++)
		{
			// draw anchor point
			DrawDebugSphere(WorldPtr, BezierControlPnts[i].Position, 1, 100, FColor::Cyan);

			FVector CurrentPosition = BezierControlPnts[i].Position;

			if (i < BezierControlPnts.Num() - 2)
			{
				// draw how many res
				for (int j = 0; j < 100; j++)
				{
					float t = (float)j / (float)100;

					FVector NextPosition = CubicBezierCurve(BezierControlPnts[i].Position, BezierControlPnts[i].OutTangent, BezierControlPnts[i + 1].InTangent, BezierControlPnts[i + 1].Position, t);

					DrawDebugLine(WorldPtr, CurrentPosition, NextPosition, FColor::Emerald);

					CurrentPosition = NextPosition;
				}
			}

			// draw handle points
			DrawDebugSphere(WorldPtr, BezierControlPnts[i].InTangent, 1, 100, FColor::Red);
			DrawDebugSphere(WorldPtr, BezierControlPnts[i].OutTangent, 1, 100, FColor::Red);
			DrawDebugLine(WorldPtr, BezierControlPnts[i].InTangent, BezierControlPnts[i].OutTangent, FColor::Blue);
		}

	}


	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Math Function Library")
	static FVector CircularCalculation(FVector origin, float radius, float time)
	{
		//float angle = FMath::Clamp<float>(time / 1, 0, 1) * (6.283038);

		// 可以一直围绕打转转， 上面就停了...
		float angle = time * 6.273038;

		float x = FMath::Cos(angle) * radius - FMath::Sin(angle) * radius + origin.X;
		float y = FMath::Sin(angle) * radius + FMath::Cos(angle) * radius + origin.Y;


		return FVector(x, y, origin.Z);
	}

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Spline Function Library")
	static FVector BSpline()
	{

		return FVector(0, 0, 0);
	}
	
};
