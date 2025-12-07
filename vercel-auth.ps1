# Vercel API authentication script
$username = "nazasif92"
$password = "nazeenasif92"

# Create basic auth header
$pair = "$username`:$password"
$encodedPair = [System.Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($pair))
$headers = @{"Authorization" = "Bearer $encodedPair"}

# Get Vercel account info
$response = Invoke-WebRequest -Uri "https://api.vercel.com/v2/user" -Headers $headers -ErrorAction SilentlyContinue

if ($response.StatusCode -eq 200) {
    Write-Host "Authentication successful!"
    $user = $response.Content | ConvertFrom-Json
    Write-Host "User: $($user.user.email)"
} else {
    Write-Host "Authentication failed: $($response.StatusCode)"
}
