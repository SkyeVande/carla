
#pragma once

#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/GnssSensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Actor/ActorDescription.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/GeoLocation.h"
#include <compiler/enable-ue4-macros.h>

#include "ImprovedGnssSensor.generated.h"

#define DIM 4

/// Gnss sensor representation
/// The actual position calculation is done one server side
UCLASS()
class CARLA_API AImprovedGnssSensor : public AGnssSensor
{
  GENERATED_BODY()

public:
  struct satellite_t {
  int PRN;
  FVector location;
  double bias;
  };

  AImprovedGnssSensor(const FObjectInitializer &ObjectInitializer);

  static FActorDefinition GetSensorDefinition();

  void Set(const FActorDescription &ActorDescription);

  // virtual void PrePhysTick(float DeltaSeconds) override;
  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaSeconds) override;

  void SetLatitudeDeviation(float Value);
  void SetLongitudeDeviation(float Value);
  void SetAltitudeDeviation(float Value);

  void SetLatitudeBias(float Value);
  void SetLongitudeBias(float Value);
  void SetAltitudeBias(float Value);

protected:

  virtual void BeginPlay() override;
  


private:

  const double mu = 3.986005*pow(10,14); //m^3/s^2
  const double Om_dot_e = 7.2921151467*pow(10,-5); //rad/s
  const double c = 299792458; //m/s
  const double F = -4.442807633*pow(10,-10);
  const double eE = .0818;
  const double Re = 6378.137;
  const double pi = 3.14159265359;

  std::map<int, double> GetPseudoranges(std::map<int, satellite_t> DetectedSatellites);
  std::map<int, satellite_t> GetVisibleSatellites();
  std::vector<std::map<std::string, double>> ReadSatellitesFromCsv();
  satellite_t calculateSatPos(std::map<std::string, double> s, double t_in);
  double NewtonRaphsonE(double M, double epsilon, double e);
  FVector ConvertToGlobalVector(double x_ecef, double y_ecef, double z_ecef);

  FVector GetVectorLocation(std::map<int, satellite_t> DetectedSatellites, std::map<int, double> Pseudoranges);

  bool NewtonRhapson(double p[], FVector S[], double B[], double epsilon);
  FVector positionUpdate(FVector4 yb, double p[], FVector S[], double B[]); 

  void getCofactor(double A[DIM][DIM], double temp[DIM][DIM], int p, int q, int n);
  double determinant(double A[DIM][DIM], int n);
  void adjoint(double A[DIM][DIM],double adj[DIM][DIM]);
  bool inverse(double A[DIM][DIM], double inverse[DIM][DIM]);

  carla::geom::GeoLocation CurrentGeoReference;
  FVector CurrentVecReference;

  float LatitudeDeviation;
  float LongitudeDeviation;
  float AltitudeDeviation;

  float LatitudeBias;
  float LongitudeBias;
  float AltitudeBias;

};
